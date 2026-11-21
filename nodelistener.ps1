$mcast = [Net.IPAddress]'239.255.254.254'
$port = 2561
$u = New-Object Net.Sockets.UdpClient($port)
$u.JoinMulticastGroup($mcast)
$ep = New-Object Net.IPEndPoint([Net.IPAddress]::Any, 0)

Write-Host 'Listening for multicast traffic on 239.255.254.254:2561...' -ForegroundColor Cyan
Write-Host ''

while($true) {
  try {
    $b = $u.Receive([ref]$ep)
    $msg = [Text.Encoding]::UTF8.GetString($b)
    $timestamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff'
    Write-Host "[$timestamp] $($ep.Address):$($ep.Port) -> $msg"
  }
  catch {
    Write-Host "Error: $_" -ForegroundColor Red
  }
}
