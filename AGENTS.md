# arduIO — Agent Instructions

Part of the RVC ecosystem. **Read [rvc-ecosystem/AGENTS.md](https://github.com/petercorke/rvc-ecosystem/blob/main/AGENTS.md) first** — it defines shared conventions: repo ownership, math invariants, dependency boundaries, git/PR workflow, code standards, tech-debt tracking. This file only adds what's specific to this repo.

| | |
|---|---|
| PyPI package | `arduio` |
| Nickname | arduIO |
| Owner | Peter Corke (`petercorke`) |
| Default branch | `main` |
| Contribution model | Branch → PR; direct push to `main` at Peter's discretion |

## Notes specific to this repo

- Two halves in one repo: a Python client (`arduio/`) and an Arduino sketch
  (`arduio_server/arduio_server.ino`) — changes to the wire protocol need both sides updated
  together.
- Used by `bdsim`'s hardware I/O blocks.
