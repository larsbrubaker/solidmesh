# Stores a crates.io API token as the CARGO_REGISTRY_TOKEN secret of larsbrubaker/solidmesh, which
# the publish workflow (.github/workflows/publish.yml) hands to `cargo publish`.
#
# The token is read with hidden input and handed straight to `gh secret set --body`, so it never
# appears on screen, in shell history or in a file (it is on gh's command line only while gh
# runs, visible to this machine's own process list). Needs the GitHub CLI
# signed in to an account that can administer the repository (`gh auth status`).
#
# Usage (PowerShell, from anywhere):
#   pwsh -File scripts/set-crates-token.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\set-crates-token.ps1

$ErrorActionPreference = 'Stop'
$repo = 'larsbrubaker/solidmesh'

if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
    throw 'The GitHub CLI (gh) is not on PATH. Install it from https://cli.github.com and sign in with `gh auth login`.'
}

$secure = Read-Host -AsSecureString "crates.io API token for $repo (input hidden)"
$bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
try {
    $token = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr)
    if ([string]::IsNullOrWhiteSpace($token)) {
        throw 'No token entered; nothing was changed.'
    }
    # Passed with --body, not piped: PowerShell adds a line ending to whatever it pipes to a
    # native program, and a CR/LF inside the token makes cargo refuse it as a header character.
    gh secret set CARGO_REGISTRY_TOKEN --repo $repo --body $token.Trim()
    if ($LASTEXITCODE -ne 0) {
        throw "gh secret set failed (exit $LASTEXITCODE); the secret was not changed."
    }
}
finally {
    [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr)
    Remove-Variable token -ErrorAction SilentlyContinue
}

Write-Host "CARGO_REGISTRY_TOKEN is set on $repo. Check with: gh secret list --repo $repo"
