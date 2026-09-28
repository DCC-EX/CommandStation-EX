$ErrorActionPreference = 'Stop'

# Host regression for the persisted EEStoreData header. The format is:
# "DCC++1" plus its terminating NUL, then three little-endian uint16 values.
$id = [Text.Encoding]::ASCII.GetBytes("DCC++1`0")
$bytes = [byte[]]::new(14)
[Array]::Copy($id, 0, $bytes, 0, $id.Length)
[BitConverter]::GetBytes([uint16]3).CopyTo($bytes, 7)
[BitConverter]::GetBytes([uint16]5).CopyTo($bytes, 9)
[BitConverter]::GetBytes([uint16]7).CopyTo($bytes, 11)

if ($bytes.Length -ne 14) { throw 'EEStoreData size changed' }
if ([Text.Encoding]::ASCII.GetString($bytes, 0, 7) -ne "DCC++1`0") {
  throw 'legacy identifier is not preserved'
}
if ([BitConverter]::ToUInt16($bytes, 7) -ne 3 -or
    [BitConverter]::ToUInt16($bytes, 9) -ne 5 -or
    [BitConverter]::ToUInt16($bytes, 11) -ne 7) {
  throw 'legacy counts did not round-trip'
}

$bytes[0] = [byte][char]'X'
if ([Text.Encoding]::ASCII.GetString($bytes, 0, 7) -eq "DCC++1`0") {
  throw 'invalid identifier accepted'
}

Write-Output 'EEStore host format test passed'
