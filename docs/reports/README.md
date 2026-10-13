# docs/reports

Agent and review reports for RocketChip (audits, research write-ups, bench analyses).

## Rules

- **File name:** `YYYY-MM-DD_<author>_<topic>.md` (for example `2026-10-11_goddard_ao_qep_audit.md`).
- **Header:** every report starts with its date, author, and scope.
- **Labels:** mark each finding as a fact (read from code, a measurement, or a cited source) or an inference (not measured or not verified).
- **Frozen once written:** do not edit an old report. A newer report supersedes an older one and names the report it supersedes.
- **Citations:** changelog entries and commit messages that act on a report cite its path.
- **Out of the repo:** scratch notes and raw tool output (logs, captures, dumps) do not go in this folder.
- **Before a public release:** reports get a cleanup pass.
