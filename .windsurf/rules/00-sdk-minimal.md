---
trigger: always_on
description: Minimal rules for the go2_ros2_sdk submodule (keep upstream compatibility; prefer minimal patches)
labels: third_party,submodule,ros2,cplusplus
---

# go2_ros2_sdk Submodule Rules (Minimal)

- When working in this submodule, still follow the BattleBang project workflow conventions (Jira/MCP + git rules).

## Jira & MCP

- Before proposing any git workflow actions (branching/commits/PRs), identify the Jira ticket ID in the form `BTB-XXX`. If unknown, ask for it.
- When the user references a task/ticket, use Jira MCP tools to fetch ticket details.
- Before making non-trivial changes, search Confluence (Space: "Kong Robot") for relevant specs/architecture.

## Git Workflow (Strict)

- Branch naming must be: `{type}/{BTB-XXX}-{short-description}`
  - Allowed `type`: `feature`, `fix`, `refactor`, `docs`, `chore`
- Commit message format must be: `[{BTB-XXX}] {message}`
- Keep commits small and scoped so they can be upstreamed or reverted safely.

## Change Policy (Submodule)

- This directory is an external dependency (git submodule). Prefer changes in the BattleBang root repo first.
- If changes are required here, keep them small and focused.
- Avoid large refactors, renames, or style-only changes.
- Keep interfaces stable; do not break downstream code without coordinating changes.
- Ensure changes remain buildable on x86 Docker and Jetson (ARM64).
- Guard Jetson-specific code paths when possible.
