param(
    [string]$m = "Update Turnback Simulator",
    [switch]$NoPush
)

$ErrorActionPreference = "Stop"
$sshKey  = "/home/honeybadger/.ssh/id_ed25519"
$vpsUser = "root"
$vpsHost = "185.164.110.65"
$vpsPath = "/root/.openclaw/workspace-volo_coding/Turnback-Simulator"
$branch  = "main"

# Git add, commit (allow empty for redeploy), push
git add -A
git diff --cached --quiet
if ($LASTEXITCODE -ne 0) {
    git commit -m $m
} else {
    Write-Host "No staged changes; skipping commit." -ForegroundColor Yellow
}
if (-not $NoPush) {
    git push origin $branch
}

# Pull latest on VPS, then rebuild the streamlit container if any python /
# Dockerfile / compose file changed.
wsl ssh -i $sshKey "$vpsUser@$vpsHost" @"
cd $vpsPath
before=`$(git rev-parse HEAD)
git pull origin $branch
after=`$(git rev-parse HEAD)
if [ "`$before" != "`$after" ] && git diff --name-only "`$before" "`$after" | grep -qE '\.py`$|^Dockerfile`$|^requirements\.txt`$|^docker-compose\.yml`$'; then
  echo 'Code changed -- rebuilding turnback container...'
  docker compose up -d --build turnback
elif ! docker ps --format '{{.Names}}' | grep -q '^turnback_simulator`$'; then
  echo 'turnback_simulator not running -- starting...'
  docker compose up -d turnback
else
  echo 'No code changes; container already running.'
fi
echo 'Deployed to $vpsPath'
"@

# Refresh portal index
wsl ssh -i $sshKey "$vpsUser@$vpsHost" "if [ -x /usr/local/bin/sync-and-update-portal.sh ]; then /usr/local/bin/sync-and-update-portal.sh; fi"

$deployedAt = Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz'
Write-Host "`nDeployed to https://turnback.voloaltro.tech at $deployedAt" -ForegroundColor Green
