<#
  Serial monitor / diagnostic for the rain firmware.
  Reads COMx, prints each line, appends to a log file, and parses SNP/EVT/REJ.

  Usage (run from project root):
    powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
    powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1 -Port COM3 -Seconds 60

  Params:
    -Port    serial port, default COM3
    -Baud    baud rate, default 115200
    -Seconds run time; 0 = run until Ctrl+C (default 0)
    -Log     log file, default <projectroot>\serial_log.txt
  Close any other program using the port first (serial assistant, etc.).

  SNP line fields (from Process_Snapshot_IfReady):
    SNP,seq,PA0peakADC,plateauRun,hardClipRun,PA1rawPeak_mV,source,use_pa1
    source: 0=PA0  1=switch-to-PA1  2=CLIP(over-range but PA1 invalid)
#>
param(
    [string]$Port    = "COM3",
    [int]   $Baud    = 115200,
    [int]   $Seconds = 0,
    [string]$Log     = ""
)

if ([string]::IsNullOrEmpty($Log)) {
    $Log = Join-Path (Split-Path -Parent $PSScriptRoot) "serial_log.txt"
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 400
$sp.NewLine = "`n"

try { $sp.Open() }
catch { Write-Host "Cannot open $Port : $($_.Exception.Message)" -ForegroundColor Red; exit 1 }

"# ==== session $(Get-Date -Format o) Port=$Port Baud=$Baud ====" | Out-File -FilePath $Log -Append -Encoding utf8
Write-Host "Opened $Port @ $Baud bps, log: $Log" -ForegroundColor Green
if ($Seconds -gt 0) { Write-Host "Running $Seconds s (Ctrl+C to stop early)" } else { Write-Host "Running until Ctrl+C" }

$snp = 0; $src0 = 0; $src1 = 0; $src2 = 0; $usep = 0
$pa0min = [int]::MaxValue; $pa0max = 0
$plateauMax = 0
$pa1min = [int]::MaxValue; $pa1max = 0
$evt = 0; $rej = 0

$deadline = if ($Seconds -gt 0) { (Get-Date).AddSeconds($Seconds) } else { [datetime]::MaxValue }

try {
    while ((Get-Date) -lt $deadline) {
        $line = $null
        try { $line = $sp.ReadLine() } catch [TimeoutException] { continue } catch { Start-Sleep -Milliseconds 50; continue }
        if ($null -eq $line) { continue }
        $line = $line.Trim()
        if ($line -eq "") { continue }

        $ts = Get-Date -Format "HH:mm:ss.fff"
        $row = "$ts  $line"
        $row | Out-File -FilePath $Log -Append -Encoding utf8

        if ($line.StartsWith("SNP,")) {
            $f = $line.Split(",")
            if ($f.Count -ge 8) {
                $snp++
                $p0 = [int]$f[2]; $run = [int]$f[3]; $p1 = [int]$f[5]; $s = $f[6]; $u = $f[7]
                if ($p0 -lt $pa0min) { $pa0min = $p0 }; if ($p0 -gt $pa0max) { $pa0max = $p0 }
                if ($run -gt $plateauMax) { $plateauMax = $run }
                if ($p1 -lt $pa1min) { $pa1min = $p1 }; if ($p1 -gt $pa1max) { $pa1max = $p1 }
                switch ($s) { "0" { $src0++ } "1" { $src1++ } "2" { $src2++ } }
                if ($u -eq "1") { $usep++ }
                $color = "Gray"
                if ($s -eq "1") { $color = "Green" } elseif ($s -eq "2") { $color = "Yellow" }
                Write-Host $row -ForegroundColor $color
            } else { Write-Host $row }
        }
        elseif ($line.StartsWith("EVT")) { $evt++; Write-Host $row -ForegroundColor Cyan }
        elseif ($line.StartsWith("REJ,")) { $rej++; Write-Host $row -ForegroundColor DarkGray }
        else { Write-Host $row }
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    if ($pa0min -eq [int]::MaxValue) { $pa0min = 0 }
    if ($pa1min -eq [int]::MaxValue) { $pa1min = 0 }
    Write-Host ""
    Write-Host "==================== SUMMARY ====================" -ForegroundColor Green
    Write-Host ("SNP lines    : {0}" -f $snp)
    Write-Host ("  source     : PA0={0}  PA1switch={1}  CLIP={2}" -f $src0, $src1, $src2)
    Write-Host ("  use_pa1=1  : {0}" -f $usep)
    Write-Host ("  PA0 peak   : min={0} max={1} (ADC; 2.0V~2481, 2.8V gate~3475, 3.3V=4095)" -f $pa0min, $pa0max)
    Write-Host ("  plateauMax : {0} (threshold PA0_PLATEAU_COUNT_TH=5)" -f $plateauMax)
    Write-Host ("  PA1 raw mV : min={0} max={1} (valid min PA1_VALID_MIN_MV=150)" -f $pa1min, $pa1max)
    Write-Host ("EVT lines    : {0}   REJ lines: {1}" -f $evt, $rej)
    Write-Host ("Log saved    : {0}" -f $Log)
    Write-Host "================================================" -ForegroundColor Green
}
