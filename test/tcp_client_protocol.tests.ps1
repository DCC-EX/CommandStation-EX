# EX-IOExpander TCP protocol/lifecycle regression tests.
#
# These tests deliberately use only the .NET socket classes available in
# PowerShell.  They exercise the wire contract used by IO_EXIOExpander_TCP.h
# and the EX-IOExpander TCP server, without requiring an Arduino board or
# PlatformIO installation.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$EXIOINIT = [byte]0xE0
$EXIOPINS = [byte]0xE9
$EXIORDAN = [byte]0xE4
$EXIORDD = [byte]0xE6
$EXIOWRD = [byte]0xE5
$EXIOERR = [byte]0xEF

function Assert-True([bool]$condition, [string]$message) {
  if (-not $condition) { throw "ASSERTION FAILED: $message" }
}

function Assert-Equal($actual, $expected, [string]$message) {
  if ($actual -ne $expected) {
    throw "ASSERTION FAILED: $message (actual=$actual expected=$expected)"
  }
}

function New-Listener {
  $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
  $listener.Start()
  return $listener
}

function Read-Exact([System.Net.Sockets.NetworkStream]$stream, [int]$length) {
  $bytes = [byte[]]::new($length)
  $offset = 0
  while ($offset -lt $length) {
    $count = $stream.Read($bytes, $offset, $length - $offset)
    if ($count -le 0) { throw 'Peer closed before a complete frame was received' }
    $offset += $count
  }
  return $bytes
}

function Write-Chunked([System.Net.Sockets.NetworkStream]$stream, [byte[]]$bytes, [int]$chunkSize) {
  $offset = 0
  while ($offset -lt $bytes.Length) {
    $count = [Math]::Min($chunkSize, $bytes.Length - $offset)
    $stream.Write($bytes, $offset, $count)
    $stream.Flush()
    $offset += $count
  }
}

function Write-Frame([System.Net.Sockets.NetworkStream]$stream, [byte]$command, [byte[]]$payload, [int]$chunkSize = 1024) {
  if ($payload.Length -gt 255) { throw 'Payload exceeds one-byte protocol length' }
  $frame = [byte[]]::new(2 + $payload.Length)
  $frame[0] = $command
  $frame[1] = [byte]$payload.Length
  if ($payload.Length -gt 0) { [Array]::Copy($payload, 0, $frame, 2, $payload.Length) }
  Write-Chunked $stream $frame $chunkSize
}

function Read-Frame([System.Net.Sockets.NetworkStream]$stream) {
  $header = Read-Exact $stream 2
  $payload = Read-Exact $stream ([int]$header[1])
  [pscustomobject]@{ Command = $header[0]; Payload = $payload }
}

function New-TcpClient([int]$port) {
  $client = [System.Net.Sockets.TcpClient]::new()
  $client.Connect([System.Net.IPAddress]::Loopback, $port)
  $client.ReceiveTimeout = 2000
  $client.SendTimeout = 2000
  return $client
}

function Invoke-Init([System.Net.Sockets.NetworkStream]$clientStream,
                     [System.Net.Sockets.NetworkStream]$serverStream,
                     [byte]$pinCount, [int]$firstVpin) {
  $payload = [byte[]]@(
    $pinCount,
    [byte]($firstVpin -band 0xff),
    [byte](($firstVpin -shr 8) -band 0xff)
  )
  Write-Frame $clientStream $EXIOINIT $payload 1
  $request = Read-Frame $serverStream
  Assert-Equal $request.Command $EXIOINIT 'init request command'
  Write-Frame $serverStream $EXIOPINS ([byte[]]@($EXIOPINS, $pinCount, 0)) 1
  return Read-Frame $clientStream
}

function Assert-PollPayload([byte[]]$payload, [byte]$command, [byte[]]$expectedState) {
  # The current server sends [CMD][DATA...].  The earlier client accepted
  # [DATA...] and must remain compatible with that representation.
  if ($payload.Length -eq $expectedState.Length) {
    Assert-True ([System.Linq.Enumerable]::SequenceEqual($payload, $expectedState)) 'raw poll payload'
    return
  }
  Assert-Equal $payload.Length ($expectedState.Length + 1) 'tagged poll payload length'
  Assert-Equal $payload[0] $command 'tagged poll command'
  $state = [byte[]]::new($expectedState.Length)
  [Array]::Copy($payload, 1, $state, 0, $state.Length)
  Assert-True ([System.Linq.Enumerable]::SequenceEqual($state, $expectedState)) 'tagged poll state'
}

$listeners = [System.Collections.Generic.List[object]]::new()
$clients = [System.Collections.Generic.List[object]]::new()
$passed = 0

try {
  # 1. Partial writes + tagged poll response, matching the server PR.
  $listener = New-Listener; $listeners.Add($listener)
  $port = ([System.Net.IPEndPoint]$listener.LocalEndpoint).Port
  $accept = $listener.AcceptTcpClientAsync()
  $client = New-TcpClient $port; $clients.Add($client)
  $server = $accept.GetAwaiter().GetResult(); $clients.Add($server)
  $clientStream = $client.GetStream(); $serverStream = $server.GetStream()
  $initResponse = Invoke-Init $clientStream $serverStream 16 800
  Assert-Equal $initResponse.Command $EXIOPINS 'init response command'
  Assert-Equal $initResponse.Payload[0] $EXIOPINS 'init response tag'
  Write-Frame $clientStream $EXIORDD ([byte[]]@()) 1
  $pollRequest = Read-Frame $serverStream
  Assert-Equal $pollRequest.Command $EXIORDD 'digital poll request command'
  Write-Frame $serverStream $EXIORDD ([byte[]]@( $EXIORDD, 0x5A )) 1
  $poll = Read-Frame $clientStream
  Assert-PollPayload $poll.Payload $EXIORDD ([byte[]]@(0x5A))
  [void]$passed++

  # 2. Backward-compatible raw poll response.
  Write-Frame $clientStream $EXIORDAN ([byte[]]@()) 1
  $pollRequest = Read-Frame $serverStream
  Assert-Equal $pollRequest.Command $EXIORDAN 'analogue poll request command'
  Write-Frame $serverStream $EXIORDAN ([byte[]]@(0x34, 0x12)) 1
  $poll = Read-Frame $clientStream
  Assert-PollPayload $poll.Payload $EXIORDAN ([byte[]]@(0x34, 0x12))
  [void]$passed++

  # 3. Error response is distinguishable from a successful acknowledgement.
  Write-Frame $clientStream $EXIOWRD ([byte[]]@(3, 1)) 1
  $request = Read-Frame $serverStream
  Assert-Equal $request.Command $EXIOWRD 'write request command'
  Write-Frame $serverStream $EXIOERR ([byte[]]@(0x02)) 1
  $errorFrame = Read-Frame $clientStream
  Assert-Equal $errorFrame.Command $EXIOERR 'error response command'
  Assert-True ($errorFrame.Command -ne [byte]0xE1) 'error must not be treated as EXIORDY'
  [void]$passed++

  # 4. Reconnect: a closed session can be replaced by a fresh handshake.
  $server.Close(); $client.Close()
  $listener.Stop(); $null = $listeners.Remove($listener)
  $listener = New-Listener; $listeners.Add($listener)
  $port = ([System.Net.IPEndPoint]$listener.LocalEndpoint).Port
  $accept = $listener.AcceptTcpClientAsync()
  $client = New-TcpClient $port; $clients.Add($client)
  $server = $accept.GetAwaiter().GetResult(); $clients.Add($server)
  $reconnectResponse = Invoke-Init $client.GetStream() $server.GetStream() 8 900
  Assert-Equal $reconnectResponse.Command $EXIOPINS 'reconnect init command'
  Assert-Equal $reconnectResponse.Payload[0] $EXIOPINS 'reconnect init tag'
  [void]$passed++

  # 5. Multiple independent sessions: each client keeps its own vPIN context.
  $listener.Stop(); $null = $listeners.Remove($listener)
  $listener = New-Listener; $listeners.Add($listener)
  $port = ([System.Net.IPEndPoint]$listener.LocalEndpoint).Port
  $acceptA = $listener.AcceptTcpClientAsync()
  $clientA = New-TcpClient $port; $clients.Add($clientA)
  $serverA = $acceptA.GetAwaiter().GetResult(); $clients.Add($serverA)
  $acceptB = $listener.AcceptTcpClientAsync()
  $clientB = New-TcpClient $port; $clients.Add($clientB)
  $serverB = $acceptB.GetAwaiter().GetResult(); $clients.Add($serverB)
  Write-Frame $clientA.GetStream() $EXIOINIT ([byte[]]@(4, 0x20, 0x03)) 1
  Write-Frame $clientB.GetStream() $EXIOINIT ([byte[]]@(4, 0x84, 0x03)) 1
  $requestA = Read-Frame $serverA.GetStream()
  $requestB = Read-Frame $serverB.GetStream()
  Assert-Equal $requestA.Payload[1] 0x20 'client A vPIN low byte'
  Assert-Equal $requestB.Payload[1] 0x84 'client B vPIN low byte'
  Write-Frame $serverA.GetStream() $EXIOPINS ([byte[]]@($EXIOPINS, 4, 0)) 1
  Write-Frame $serverB.GetStream() $EXIOPINS ([byte[]]@($EXIOPINS, 4, 0)) 1
  Assert-Equal (Read-Frame $clientA.GetStream()).Command $EXIOPINS 'client A response'
  Assert-Equal (Read-Frame $clientB.GetStream()).Command $EXIOPINS 'client B response'
  [void]$passed++

  "PASS: $passed TCP client protocol/lifecycle tests"
}
finally {
  foreach ($clientToClose in $clients) { if ($null -ne $clientToClose) { $clientToClose.Dispose() } }
  foreach ($listenerToClose in $listeners) { if ($null -ne $listenerToClose) { $listenerToClose.Stop() } }
}
