@AK_GUIDELINES.md
@PROTECTED_FILES.md
@SESSION_CHECKLIST.md

@../README.md
@../standards/CODING_STANDARDS.md
@../standards/DEBUG_OUTPUT.md
@../standards/GIT_WORKFLOW.md
@../standards/STANDARDS_DEVIATIONS.md

# Claude-Specific Context (check on every session)
@LESSONS_LEARNED.md
@DEBUG_PROBE_NOTES.md
@../docs/MULTICORE_RULES.md

# Cross-Context Communication
# Use AGENT_WHITEBOARD.md for:
# - Flagging issues for review (per CROSS_AGENT_REVIEW.md)
# - Communicating between Claude sessions (context window handoffs)
# - Noting work-in-progress that spans multiple sessions
@../AGENT_WHITEBOARD.md

# Plan Mode Council Review
# Before calling ExitPlanMode:
# 1. Ask the user which council personas to use (suggest relevant ones from COUNCIL_PROCESS.md)
# 2. Once confirmed, spawn a Task agent to run the council review of the drafted plan
# 3. The agent receives the persona definitions + the plan
# 4. Attach the council's verdict (consensus, dissents, red flags) to the plan
# Skip the entire review only if the user explicitly requests no council review.
@../COUNCIL_PROCESS.md
