#!/usr/bin/env bash
# 본선 경쟁 3팀 GitHub 델타 추적 — 마지막 실행 이후 무엇이 바뀌었는지만 보여준다.
#
#   ./경쟁3팀추적.sh              # 지난 실행 이후 델타 (기본)
#   ./경쟁3팀추적.sh --full       # 상태 무시하고 각 브랜치 최근 커밋
#   ./경쟁3팀추적.sh --evidence   # 커밋 본문까지 받아 실차 증거 키워드 표시
#   ./경쟁3팀추적.sh --log        # 결과를 참고레포분석/추적로그.md 에 append
#   ./경쟁3팀추적.sh --reset      # 상태 초기화 (다음 실행이 전량 새 것으로 보임)
#
# 왜 3팀인가: 본선 22팀 중 공개 레포가 있는 팀은 6팀뿐이고, 그중 실차가 실제로
# 움직이는 건 이 3팀이다. 나머지는 정지했거나 교육 패키지 사본이다.
# 근거: 참고레포분석/본선팀-매핑-0818.md
#
# 왜 검색 API 를 안 쓰나: search 는 30회/분 제한이라 스로틀 없이 돌리면 에러 문자열이
# 결과로 섞여 들어온다(조용한 오염). 이 스크립트는 코어 API(5,000/시)만 쓴다.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="$ROOT/참고레포분석/.추적상태"
LOG_FILE="$ROOT/참고레포분석/추적로그.md"

# 팀명|레포|한 줄 메모 (매핑 근거는 본선팀-매핑-0818.md)
TARGETS=(
  "ERI-CAR(한양ERICA)|doldolmeng2/2026-kookmin-contest|코드 최상급·실차 커밋 08-12 이후 정지"
  "KURiver|mastic-choi/UMK|유일하게 매일 실차 주행·하드웨어 불안정"
  "아주INFO|LunaGunwoo/xycar_ws_mgw|guided 수집 단계·미션 로직 0"
)

# 실차에서만 나올 수 있는 단어들 — 이게 뜨면 그 팀이 차를 굴렸다는 뜻이다
EVIDENCE_RE='실차|완주|주행|트랙|충돌|벽|이탈|bag|녹화|실측|랩|real.?vehicle|lap|track'

FULL=0; EVIDENCE=0; LOG=0
for a in "$@"; do
  case "$a" in
    --full)     FULL=1 ;;
    --evidence) EVIDENCE=1 ;;
    --log)      LOG=1 ;;
    --reset)    rm -rf "$STATE_DIR"; echo "상태 초기화 완료: $STATE_DIR"; exit 0 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *)          echo "알 수 없는 인자: $a"; exit 1 ;;
  esac
done

command -v gh >/dev/null || { echo "gh CLI 가 필요하다: brew install gh"; exit 1; }
gh auth status >/dev/null 2>&1 || { echo "gh auth login 이 필요하다"; exit 1; }
mkdir -p "$STATE_DIR"

out() { printf '%s\n' "$*"; [ "$LOG" = 1 ] && printf '%s\n' "$*" >> "$LOG_FILE"; }

# 커밋 목록 출력 (인자: repo, jq로 뽑은 커밋 JSON 배열 경로)
print_commits() {
  local repo="$1" file="$2" indent="$3"
  jq -r '.[] | "\(.commit.author.date[0:16])|\(.commit.author.name)|\(.sha[0:7])|\(.commit.message | split("\n")[0])"' "$file" 2>/dev/null \
  | while IFS='|' read -r date who sha subj; do
      local mark=""
      printf '%s' "$subj" | grep -qiE "$EVIDENCE_RE" && mark=" ★실차?"
      out "${indent}${date}  ${who}  ${sha}  ${subj:0:76}${mark}"
      if [ "$EVIDENCE" = 1 ]; then
        gh api "repos/$repo/commits/$sha" --jq '.commit.message' </dev/null 2>/dev/null \
          | tail -n +2 | grep -iE "$EVIDENCE_RE" | head -3 \
          | while read -r l; do out "${indent}    │ ${l:0:100}"; done
      fi
    done
}

STAMP="$(date '+%Y-%m-%d %H:%M %Z')"
out ""
out "# 경쟁 3팀 델타 — $STAMP"
[ "$FULL" = 1 ] && out "(--full: 상태 무시, 브랜치별 최근 5커밋)"

CHANGED_TOTAL=0
for t in "${TARGETS[@]}"; do
  team="${t%%|*}"; rest="${t#*|}"; repo="${rest%%|*}"; note="${rest#*|}"
  key="$(printf '%s' "$repo" | tr '/' '_')"
  state="$STATE_DIR/$key.tsv"
  tmp="$(mktemp)"; cur="$(mktemp)"

  baseline=0; [ -f "$state" ] || baseline=1

  meta=$(gh api "repos/$repo" --jq '"\(.pushed_at)"' </dev/null 2>/dev/null)
  if [ -z "$meta" ]; then out ""; out "## $team — $repo"; out "  ⚠️ 조회 실패(레포 삭제/비공개 전환?)"; continue; fi

  gh api "repos/$repo/branches?per_page=100" --jq '.[] | "\(.name)\t\(.commit.sha)"' </dev/null 2>/dev/null > "$cur"

  out ""
  out "## $team — $repo"
  out "  마지막 push: $meta   ($note)"

  changed=0
  while IFS=$'\t' read -r br sha; do
    [ -z "$br" ] && continue
    old=""
    [ -f "$state" ] && old=$(awk -F'\t' -v b="$br" '$1==b{print $2}' "$state")

    if [ "$baseline" = 1 ] && [ "$FULL" != 1 ]; then
      continue                      # 첫 실행 = 기준선만 저장(덤프하면 수백 줄)
    elif [ "$FULL" = 1 ]; then
      out "  ─ [$br] 최근 커밋"
      gh api "repos/$repo/commits?sha=$sha&per_page=5" </dev/null 2>/dev/null > "$tmp"
      print_commits "$repo" "$tmp" "      "; changed=1
    elif [ -z "$old" ]; then
      out "  🆕 브랜치 신설: $br"
      gh api "repos/$repo/commits?sha=$sha&per_page=5" </dev/null 2>/dev/null > "$tmp"
      print_commits "$repo" "$tmp" "      "; changed=1
    elif [ "$old" != "$sha" ]; then
      out "  🔁 [$br] ${old:0:7} → ${sha:0:7}"
      gh api "repos/$repo/compare/$old...$sha" --jq '.commits' </dev/null 2>/dev/null > "$tmp"
      if [ -s "$tmp" ] && [ "$(jq 'length' "$tmp" 2>/dev/null)" != "0" ]; then
        print_commits "$repo" "$tmp" "      "
      else
        out "      (force-push 또는 히스토리 재작성 — compare 불가)"
      fi
      changed=1
    fi
  done < "$cur"

  # 사라진 브랜치
  if [ -f "$state" ] && [ "$FULL" != 1 ]; then
    while IFS=$'\t' read -r br sha; do
      [ -z "$br" ] && continue
      grep -q "^$br	" "$cur" || { out "  🗑  브랜치 삭제됨: $br"; changed=1; }
    done < "$state"
  fi

  if [ "$baseline" = 1 ] && [ "$FULL" != 1 ]; then
    out "  📌 기준선 저장 ($(wc -l < "$cur" | tr -d ' ')개 브랜치). 다음 실행부터 델타만 표시된다"
    out "     최근 브랜치: $(sort -t$'\t' -k1,1 "$cur" | cut -f1 | tr '\n' ' ' | cut -c1-90)"
    changed=0
  fi
  [ "$changed" = 0 ] && [ "$baseline" != 1 ] && out "  변화 없음"
  [ "$changed" = 1 ] && CHANGED_TOTAL=$((CHANGED_TOTAL+1))

  cp "$cur" "$state"
  rm -f "$tmp" "$cur"
done

out ""
out "— 변화 있는 팀: $CHANGED_TOTAL/3.  ★실차? 표시는 커밋 제목에 실차 관련 단어가 있다는 뜻(자동 판정 아님)."
if [ "$LOG" = 1 ]; then echo "로그 append: $LOG_FILE"; fi
exit 0
