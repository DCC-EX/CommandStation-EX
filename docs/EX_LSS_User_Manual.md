# EX_LSS Line Side Sound System User Manual

Welcome to the user manual for the **EX_LSS Line Side Sound System**. The EX_LSS is a centralized, high-fidelity sound system for model railway layouts, designed to replace legacy setups (like connected UARTs and individual DFPlayer modules). 

Powered by a Raspberry Pi RP2350 microcontroller, the system offers direct I2C end-to-end control, a central microSD card, a flexible matrix mixer, fading/panning, layout automation scripting, and comprehensive diagnostics.

---

## 1. System Overview & Key Features

The EX_LSS engine splits its duties between two processing cores to guarantee deterministic, glitch-free audio performance:
*   **Logistics & Supervision (Core 0):** Manages the microSD card filesystem (FatFs), handles I2C command processing from the DCC-EX Command Station, runs layout automation scripts, controls faders, drives optional OLED displays, and acts as the system watchdog.
*   **Deterministic DSP Engine (Core 1):** Instantly decodes up to 16 concurrent MP3 files and mixes them into 8 physical mono output channels (expandable to 8 amplifiers/speakers).

### Key Features:
*   **Centralized microSD storage:** One card stores all audio files and sequence scripts for the board.
*   **Matrix Mixer (16x8):** Mix any or all of the 16 audio streams to any of the 8 physical speakers simultaneously.
*   **Instant-Start Cache:** Pre-caches the first 16KB of files in external PSRAM to bypass microSD seek latencies, allowing lag-free loops and triggers.
*   **Rich Fading Engine:** Perform smooth Doppler sweeps, panning, or ambient cross-fades using Linear, Logarithmic, Sigmoid (S-Curve), or Equal Power volume curves.
*   **Local Sequence Scripting:** Execute up to 4 concurrent automation scripts locally on the sound board.
*   **Crash Logging & Safe Mode:** Features post-mortem register dumping and a "Three Strikes" Safe Mode to protect against infinite reboot loops.

---

## 2. USB Command Line Interface (CLI) Reference

The LSS firmware features an interactive command interpreter via its USB Serial port, allowing you to test sound files, configure hardware, and debug scripts before writing your EX-Rail automation.

### Command Format
All CLI commands must be framed with `<` and `>` (e.g., `<R FW>`). Commands and parameters are case-sensitive. Space ` ` is used to separate parameters.

### 2.1 Global Commands (`<D SYS ...>`)
These commands control or modify system-wide configurations:

| Command | Description |
| :--- | :--- |
| `<D help>` | Prints a quick-reference help menu to the USB terminal. |
| `<D SYS reset 1>` | Triggers a soft reboot of the sound engine (flushes caches, unloads slots, remounts SD card). |
| `<D SYS mute 1\|0>` | `1` = Global mute ON (fades out output without pausing playback). `0` = Global mute OFF. |
| `<D SYS oled test 1>` | Sends an OLED test message to the CLI log/OLED display. |
| `<D SYS i2c debug 1\|0>` | `1` = Enables logging of all I2C transactions to a 64KB debug tap in PSRAM. `0` = Disables logging. |
| `<D SYS i2c address [address]>` | Saves a new I2C base address (e.g., `0x40`) to Flash EEPROM and reboots. |
| `<D SYS oled page [0-5]>` | Sets the active OLED display page (`0` = Auto Cycle, `1` = Banner, `2` = Dashboard, `3` = Slots 0-7, `4` = Slots 8-15, `5` = Script Status). |
| `<D SYS factory reset>` | Wipes the Flash EEPROM back to defaults (restoring address `0x40`) and reboots. |
| `<D SYS clear crash>` | Wipes saved crash logs from Flash memory. **Must be run to unlock the system if Safe Mode is triggered.** |
| `<D config oled [width] [height]>` | Configures OLED dimensions (`width` = `0` (disabled), `128` (SSD1306), `132` (SH1106); `height` = `32`\|`64`). |
| `<D RUN SCRIPT [id] [engine_id]>` | Runs script ID `[id]` (e.g. `001` runs `001_sequence.txt`) on engine `[engine_id]` (0 to 3). Runs `startup.txt` if ID is omitted. |
| `<D STOP SCRIPT [engine_id]>` | Instantly halts the script running on engine `[engine_id]` (0 to 3). |
| `<D SCRIPT debug verbose 1\|0>` | `1` = Enables verbose script execution logging. `0` = Errors only. |

### 2.2 Slot-Specific Commands (`<D SLOT ...>`)
Slot commands are targeted to a specific audio slot `[slot]` (integer `0` to `15`):

| Command | Description |
| :--- | :--- |
| `<D SLOT [slot] LOAD [file_id]>` | Loads a file ID (e.g., `012` for `012_horn.mp3`) from the SD card, maps sectors, and primes the cache. |
| `<D SLOT [slot] FLUSH>` | Instantly aborts playback, purges the slot buffers, and unassigns the file. |
| `<D SLOT [slot] OUT [channel] [volume]>` | Routes slot output to speaker `[channel]` (0-7) at `[volume]` (0-255). |
| `<D SLOT [slot] PLAY CONTROL [flags]>` | Direct control over playback state. Flags can be combined:<br>`P` = Play/Start (5ms pop slew)<br>`S` = Stop (5ms pop slew)<br>`U` = Pause (freezes decoder)<br>`L` = Loop infinitely (seamless repeat) |
| `<D SLOT [slot] FADE [channel] [vol] [ticks] [curve]>` | Fades speaker volume in the background (non-blocking).<br>`[channel]`: 0-7<br>`[vol]`: target volume (0-255)<br>`[ticks]`: duration in 10ms intervals (1 to 30000; e.g. 400 = 4s)<br>`[curve]`: `LINEAR`, `LOG`, `S-CURVE`, or `EQUAL_POWER`. |

### 2.3 Diagnostic & Status Commands (`<R ...>`)
Query system state and print formatted diagnostic data to the USB console:

| Command | Description |
| :--- | :--- |
| `<R SYS STATUS>` | Returns a complete health dashboard of SD card, PSRAM, cores load %, and active script engines. |
| `<R FW>` | Prints active firmware version (Major.Minor). |
| `<R SHOW FILES>` | Lists all assigned slots with their loaded file numbers and full filenames. |
| `<R SHOW FILES [slot]>` | Prints filename loaded in the specified slot. |
| `<R SLOT [slot] PLAY CONTROL>` | Returns the human-readable playback state (e.g. `Slot [x] file [id] playing in Loop`). |
| `<R SLOT [slot] STATUS>` | Detailed slot telemetry including sample rate, bitrate, channels, and decoded error flags. |
| `<D SHOW REGISTERS>` | Dumps current values of all Global I2C Registers. |
| `<D SHOW REGISTERS [slot]>` | Dumps current values of all Indexed Registers for a specific slot. |
| `<D SHOW CRASH DATA>` | Prints post-mortem register logs (PC, LR, SP, Core ID) if the board crashed. |
| `<D SHOW MIXER>` | Displays a matrix mixer cross-point routing map and channel status. |
| `<D SHOW MIXER [0-7]>` | Details slot mapping, target volumes, and real-time volume slewing for a specific speaker channel. |

---

## 3. Local Sequence Scripting

Layout designers can write self-contained sequence files (named `[3-digit ID]*.txt`, such as `002_crickets.txt` or `startup.txt`) on the microSD card. The board runs up to 4 scripts concurrently in its background engines.

### Scripting Sandbox Constraints
To protect the integrity of the sound board and prevent crashes:
*   Scripts **cannot** call system command groups (`<D SYS ...>`, `<D config ...>`), preventing reboot loops and flash wearing.
*   Scripts **cannot** issue read/status diagnostic commands (`<R ...>`, `<D SHOW ...>`), avoiding CPU waste.
*   Scripts **cannot** launch or stop other scripts (`<D RUN SCRIPT>`, `<D STOP SCRIPT>`).

### Script-Only Commands
These commands are dedicated to sequence control inside files:

| Command | Description |
| :--- | :--- |
| `<D WAIT [ms]>` | Pauses script execution for the specified milliseconds (0 to 300,000). |
| `<D WAIT RANDOM [min] [max]>` | Pauses for a random time between `[min]` and `[max]` (max 300,000 ms), driven by the RP2350's hardware Random Number Generator (RNG). |
| `<D WAIT SLOT [slot] FINISHED>` | Blocks script execution until the target audio slot finishes playing or errors out. |
| `<D SLOT [slot] PLAY [count]>` | Plays the slot sound exactly `[count]` times (1 to 255) sequentially before stopping. |

---

## 4. EX-Rail Command Reference

The EX_LSS commands listed below are available directly inside your EX-Rail automation scripts (`myAutomation.h`). They act as high-level macros that map to the standard DCC-EX `ANOUT` (analogue write) command under the hood, targeting the virtual pins mapped to your LSS device slots.

When configuring the LSS in your Command Station HAL setup, you assign it a block of virtual pins (typically 16 pins, one for each slot, starting at a base `vpin`). 
*   **Slot VPin (`vpin`):** Represents the slot itself (0 to 15) relative to the base pin (e.g. `base_pin + slot`).

### LSS EX-Rail Macros

#### `LSS_LOAD(vpin, file_id)`
Pre-loads an audio file number (0–999) from the microSD card into the specified slot VPin's cache.
*   **vpin:** Slot virtual pin.
*   **file_id:** 3-digit audio file ID.
*   *Maps to:* `ANOUT(vpin, file_id, 0, 1)`

#### `LSS_FLUSH(vpin)`
Aborts playback on the slot, empties its PSRAM ring buffers, and unassigns the loaded file.
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 2)`

#### `LSS_PLAY(vpin)`
Starts playback of the loaded file in the designated slot (plays once and stops).
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 1, 0, 3)`

#### `LSS_PLAY_LOOP(vpin)`
Starts playback of the loaded file in the designated slot, looping it infinitely.
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 4)`

#### `LSS_STOP(vpin)`
Gracefully stops playback on the slot VPin (applies a 5ms anti-pop fade).
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 5)`

#### `LSS_PAUSE(vpin)`
Freezes playback on the slot VPin, retaining the current track position.
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 6)`

#### `LSS_RESUME(vpin)`
Resumes playback on the paused slot VPin.
*   **vpin:** Slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 7)`

#### `LSS_VOLUME(vpin, channel, volume)`
Sets the volume level for a specific physical speaker output channel.
*   **vpin:** Slot virtual pin.
*   **channel:** Speaker output channel (0–7).
*   **volume:** Level from 0 (silent) to 255 (maximum).
*   *Maps to:* `ANOUT(vpin, volume, channel, 8)`

#### `LSS_FADE(vpin, channel, target_volume, duration_ticks, curve)`
Initiates a background volume transition on the specified speaker channel.
*   **vpin:** Slot virtual pin.
*   **channel:** Speaker output channel (0–7).
*   **target_volume:** Target volume level (0–255).
*   **duration_ticks:** Duration in 10ms intervals (1–30000).
*   **curve:** Mathematical fade curve:
    *   `0` = Linear (constant rate, e.g., machinery sound transitions).
    *   `1` = Logarithmic (natural human hearing taper for distance simulations).
    *   `2` = S-Curve (Sigmoid curve, best for locomotive flyby/Doppler effects).
    *   `3` = Equal Power (Square-Root curve, best for environmental cross-fading).
*   *Maps to:* `ANOUT(vpin, duration_ticks, target_volume, (channel << 12) \| (curve << 8) \| 9)`

#### `LSS_GLOBAL_RESET(vpin)`
Instructs the sound board at the given VPin to execute a system soft reset.
*   **vpin:** Any LSS slot virtual pin.
*   *Maps to:* `ANOUT(vpin, 0, 0, 10)`

#### `LSS_GLOBAL_MUTE(vpin, mute)`
Mutes or unmutes all audio playback globally across the LSS board.
*   **vpin:** Any LSS slot virtual pin.
*   **mute:** `1` to mute, `0` to restore.
*   *Maps to:* `ANOUT(vpin, mute, 0, 11)`

#### `LSS_OLED_PAGE(vpin, page)`
Sets the active page on the LSS board's OLED screen.
*   **vpin:** Any LSS slot virtual pin.
*   **page:** OLED screen index (0–5).
*   *Maps to:* `ANOUT(vpin, page, 0, 12)`

#### `LSS_RUN_SCRIPT(vpin, script_id, engine_id)`
Triggers execution of layout sequence script `script_id` on the board's engine `engine_id`.
*   **vpin:** Any LSS slot virtual pin.
*   **script_id:** 3-digit sequence ID (0–999, or `0xFFFF` for `startup.txt`).
*   **engine_id:** Script engine channel (0–3).
*   *Maps to:* `ANOUT(vpin, script_id, engine_id, 13)`

#### `LSS_STOP_SCRIPT(vpin, engine_id)`
Halts script execution on the specified engine ID.
*   **vpin:** Any LSS slot virtual pin.
*   **engine_id:** Script engine channel (0–3).
*   *Maps to:* `ANOUT(vpin, 0, engine_id, 14)`
