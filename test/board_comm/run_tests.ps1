$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '../..')
$output = Join-Path $env:TEMP 'rp27_board_comm_test.exe'

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I (Join-Path $PSScriptRoot 'stubs') `
    -I (Join-Path $repo 'Application/ProtocolLayer') `
    (Join-Path $PSScriptRoot 'test_board_comm.c') `
    (Join-Path $repo 'Application/ProtocolLayer/board_comm.c') `
    -o $output

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $output
exit $LASTEXITCODE
