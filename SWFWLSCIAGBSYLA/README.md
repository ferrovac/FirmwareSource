# Changelog

All notable changes to this project will be documented in this file.


## [Unreleased]
- Controll if LN2 is connected and throw an error after 10min when temp in dewer is not bellow 0°C while autofill is on. (To prevent valve overheating) 

## [0.2.1] - 2025-04-04
### Added
- Added leak detection in glove box. If the vacuum does not reach 70000Pa within 30 seconds we abort the pumping procedure.
### Fixed
- Vent/Pump button on keypanel now has to be pressed for 1 sec to trigger venting/pumping
- Typo in Vent Error message 'valce' -> 'valve'

## [0.2.0] - 2024-03-19
### Added
- Added a text-box which labels the main vacuum vessel for clarity.
[Release: 24900_G](https://github.com/ferrovac/FirmwareSource/releases/tag/v0.2.0)
## [0.1.2] - 2024-01-25
### Added
- Buffer pressure gauge now shows "-URange-" if the pressure is better than 1E-4 mbar.

[Release: 24900_F](https://github.com/ferrovac/GLOVEBOX/releases/tag/v0.1.1.2_F)
## [0.1.1] - 2024-01-24
### Fixed
- Fixed an issue that caused the pressure in the Buffer to spike, when the CLL is switched to pumping after venting.
## [0.1.0] - 2024-01-23
- Initial release. Tested in house.
