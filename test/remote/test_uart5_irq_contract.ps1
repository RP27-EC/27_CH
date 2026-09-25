$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$interrupts = Get-Content -Raw (Join-Path $repo 'Core\Src\stm32h7xx_it.c')

function Assert-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

$uart5HandlerPattern = 'void\s+UART5_IRQHandler\s*\(void\)\s*\{(?<body>[\s\S]*?)\n\}'
if ($interrupts -notmatch $uart5HandlerPattern) {
    throw 'UART5_IRQHandler is missing'
}

$uart5Body = $Matches['body']
Assert-Match $uart5Body 'HAL_UART_IRQHandler\s*\(\s*&huart5\s*\)' `
    'UART5_IRQHandler must dispatch HAL UART interrupts'
Assert-Match $uart5Body 'DRV_UART_IRQHandler\s*\(\s*&huart5\s*\)' `
    'UART5_IRQHandler must dispatch UART5 IDLE/DMA data to the project UART driver'

Write-Host 'UART5 remote IRQ contract passed.'
