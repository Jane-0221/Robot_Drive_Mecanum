param(
  [float]$DistanceMm = 100.0,
  [string]$ElfPath = ".\build\Omni_damiao.elf",
  [int]$TelnetPort = 50002,
  [string]$OpenOcdPath = "",
  [string]$NmPath = "",
  [switch]$KeepOpenOcd
)

$ErrorActionPreference = "Stop"

function Resolve-Executable {
  param(
    [string]$ConfiguredPath,
    [string]$CommandName,
    [string[]]$Candidates
  )

  if ($ConfiguredPath -ne "") {
    if (Test-Path $ConfiguredPath) {
      return (Resolve-Path $ConfiguredPath).Path
    }
    throw "Tool not found: $ConfiguredPath"
  }

  $cmd = Get-Command $CommandName -ErrorAction SilentlyContinue
  if ($null -ne $cmd) {
    return $cmd.Source
  }

  foreach ($candidate in $Candidates) {
    if (Test-Path $candidate) {
      return $candidate
    }
  }

  throw "Tool not found: $CommandName"
}

function Resolve-Symbols {
  param(
    [string]$NmExe,
    [string]$Elf,
    [string[]]$Names
  )

  $symbols = @{}
  $nmLines = & $NmExe -n $Elf
  foreach ($name in $Names) {
    $line = $nmLines | Where-Object { $_ -match "\s$name$" } | Select-Object -First 1
    if ($null -eq $line) {
      throw "Symbol not found in ELF: $name"
    }
    if ($line -notmatch "^([0-9a-fA-F]+)\s+\w\s+$name$") {
      throw "Cannot parse symbol line: $line"
    }
    $symbols[$name] = [Convert]::ToUInt32($Matches[1], 16)
  }
  return $symbols
}

function Test-TcpPort {
  param([int]$Port)

  $client = New-Object System.Net.Sockets.TcpClient
  try {
    $async = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
    if (-not $async.AsyncWaitHandle.WaitOne(300)) {
      return $false
    }
    $client.EndConnect($async)
    return $true
  }
  catch {
    return $false
  }
  finally {
    $client.Close()
  }
}

function Connect-OpenOcdTelnet {
  param([int]$Port)

  for ($i = 0; $i -lt 50; $i++) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
      $client.Connect("127.0.0.1", $Port)
      $client.NoDelay = $true
      Start-Sleep -Milliseconds 100
      [void](Read-OpenOcdAvailable $client)
      return $client
    }
    catch {
      $client.Close()
      Start-Sleep -Milliseconds 200
    }
  }

  throw "Cannot connect to OpenOCD telnet port $Port"
}

function Read-OpenOcdAvailable {
  param([System.Net.Sockets.TcpClient]$Client)

  $stream = $Client.GetStream()
  $buffer = New-Object byte[] 4096
  $text = ""
  Start-Sleep -Milliseconds 80
  while ($stream.DataAvailable) {
    $count = $stream.Read($buffer, 0, $buffer.Length)
    if ($count -le 0) {
      break
    }
    $text += [System.Text.Encoding]::ASCII.GetString($buffer, 0, $count)
    Start-Sleep -Milliseconds 20
  }
  return $text
}

function Invoke-OpenOcd {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [string]$Command
  )

  $stream = $Client.GetStream()
  $bytes = [System.Text.Encoding]::ASCII.GetBytes($Command + "`n")
  $stream.Write($bytes, 0, $bytes.Length)
  $stream.Flush()
  $output = Read-OpenOcdAvailable $Client
  if ($output -match "invalid|failed|error") {
    throw "OpenOCD command failed: $Command`n$output"
  }
  return $output
}

function Read-Byte {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [uint32]$Address
  )

  $out = Invoke-OpenOcd $Client ("mdb 0x{0:x8} 1" -f $Address)
  if ($out -notmatch ":\s*([0-9a-fA-F]{2})") {
    throw "Cannot parse mdb output: $out"
  }
  return [Convert]::ToByte($Matches[1], 16)
}

function Read-Word {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [uint32]$Address
  )

  $out = Invoke-OpenOcd $Client ("mdw 0x{0:x8} 1" -f $Address)
  if ($out -notmatch ":\s*([0-9a-fA-F]{8})") {
    throw "Cannot parse mdw output: $out"
  }
  return [Convert]::ToUInt32($Matches[1], 16)
}

function Write-Byte {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [uint32]$Address,
    [byte]$Value
  )

  [void](Invoke-OpenOcd $Client ("mwb 0x{0:x8} 0x{1:x2}" -f $Address, $Value))
}

function Write-Float {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [uint32]$Address,
    [float]$Value
  )

  $word = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$Value), 0)
  [void](Invoke-OpenOcd $Client ("mww 0x{0:x8} 0x{1:x8}" -f $Address, $word))
}

function Read-Float {
  param(
    [System.Net.Sockets.TcpClient]$Client,
    [uint32]$Address
  )

  $word = Read-Word $Client $Address
  return [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$word), 0)
}

$elfFullPath = (Resolve-Path $ElfPath).Path
$nmExe = Resolve-Executable $NmPath "arm-none-eabi-nm" @(
  "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-nm.exe",
  "C:\Users\Godwin\13.2 Rel1\bin\arm-none-eabi-nm.exe"
)
$openOcdExe = Resolve-Executable $OpenOcdPath "openocd" @(
  "C:\msys64\ucrt64\bin\openocd.exe",
  "C:\ST\STM32CubeCLT_1.21.0\OpenOCD\bin\openocd.exe"
)

$symbols = Resolve-Symbols $nmExe $elfFullPath @(
  "rs485_lift_calibration_cmd_debug",
  "rs485_lift_calibration_state_debug",
  "rs485_lift_calibration_command_mm_debug",
  "rs485_lift_calibration_actual_mm_debug",
  "rs485_lift_calibration_old_units_per_mm_debug",
  "rs485_lift_calibration_new_units_per_mm_debug",
  "rs485_lift_calibration_end_position_mm_debug",
  "rs485_lift_calibration_error_debug"
)

$startedOpenOcd = $false
$openOcdProcess = $null
$client = $null

try {
  if (-not (Test-TcpPort $TelnetPort)) {
    $scriptDir = "C:\msys64\ucrt64\share\openocd\scripts"
    if (-not (Test-Path $scriptDir)) {
      $scriptDir = ""
    }

    $args = @()
    if ($scriptDir -ne "") {
      $args += @("-s", $scriptDir)
    }
    $args += @(
      "-c", "adapter driver cmsis-dap",
      "-c", "cmsis_dap_vid_pid 0xfaed 0x4870",
      "-c", "transport select swd",
      "-f", "target/stm32h7x.cfg",
      "-c", "adapter speed 100",
      "-c", "gdb_port disabled",
      "-c", "tcl_port disabled",
      "-c", "telnet_port $TelnetPort",
      "-c", "init"
    )

    $openOcdProcess = Start-Process -FilePath $openOcdExe -ArgumentList $args -PassThru -WindowStyle Hidden
    $startedOpenOcd = $true
  }

  $client = Connect-OpenOcdTelnet $TelnetPort

  Write-Host "Calibration distance command: $DistanceMm mm"
  Write-Host "Do not press other lift controls during calibration."

  Write-Byte $client $symbols["rs485_lift_calibration_cmd_debug"] 3
  Start-Sleep -Milliseconds 300
  Write-Float $client $symbols["rs485_lift_calibration_command_mm_debug"] $DistanceMm
  Write-Float $client $symbols["rs485_lift_calibration_actual_mm_debug"] 0.0
  Write-Byte $client $symbols["rs485_lift_calibration_cmd_debug"] 1

  $lastState = 255
  $lastPositionPrint = Get-Date
  $deadline = (Get-Date).AddSeconds(45)
  while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 300
    $state = Read-Byte $client $symbols["rs485_lift_calibration_state_debug"]
    if ($state -ne $lastState) {
      Write-Host "State: $state"
      $lastState = $state
    }
    if (((Get-Date) - $lastPositionPrint).TotalSeconds -ge 1.0) {
      $programNow = Read-Float $client $symbols["rs485_lift_calibration_end_position_mm_debug"]
      Write-Host ("Program position: {0:n3} mm" -f $programNow)
      $lastPositionPrint = Get-Date
    }
    if ($state -eq 3) {
      break
    }
    if ($state -eq 6) {
      $err = Read-Word $client $symbols["rs485_lift_calibration_error_debug"]
      throw ("Calibration stopped with firmware error 0x{0:x8}" -f $err)
    }
  }

  $state = Read-Byte $client $symbols["rs485_lift_calibration_state_debug"]
  if ($state -ne 3) {
    throw "Timed out waiting for WAIT_ACTUAL state. Current state: $state"
  }

  $programEnd = Read-Float $client $symbols["rs485_lift_calibration_end_position_mm_debug"]
  Write-Host ("Program position after move: {0:n3} mm" -f $programEnd)
  $actualText = Read-Host "Enter measured lift distance in mm"
  $actualMm = [float]$actualText
  if ($actualMm -le 0.0) {
    throw "Measured distance must be greater than 0"
  }

  Write-Float $client $symbols["rs485_lift_calibration_actual_mm_debug"] $actualMm
  Write-Byte $client $symbols["rs485_lift_calibration_cmd_debug"] 2

  $deadline = (Get-Date).AddSeconds(10)
  while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 200
    $state = Read-Byte $client $symbols["rs485_lift_calibration_state_debug"]
    if ($state -eq 5) {
      break
    }
    if ($state -eq 6) {
      $err = Read-Word $client $symbols["rs485_lift_calibration_error_debug"]
      throw ("Calibration apply failed with firmware error 0x{0:x8}" -f $err)
    }
  }

  $state = Read-Byte $client $symbols["rs485_lift_calibration_state_debug"]
  if ($state -ne 5) {
    throw "Timed out waiting for DONE state. Current state: $state"
  }

  $oldUnits = Read-Float $client $symbols["rs485_lift_calibration_old_units_per_mm_debug"]
  $newUnits = Read-Float $client $symbols["rs485_lift_calibration_new_units_per_mm_debug"]
  $endMm = Read-Float $client $symbols["rs485_lift_calibration_end_position_mm_debug"]

  Write-Host ("Calibration done. old units/mm={0:n3}, new units/mm={1:n3}, firmware height={2:n3} mm" -f $oldUnits, $newUnits, $endMm)
}
finally {
  if ($null -ne $client) {
    $client.Close()
  }
  if ($startedOpenOcd -and (-not $KeepOpenOcd) -and ($null -ne $openOcdProcess)) {
    Stop-Process -Id $openOcdProcess.Id -Force -ErrorAction SilentlyContinue
  }
}
