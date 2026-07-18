"""
assistant.py — AI assistant layer for the Apples & Oranges sorter.

Turns natural language (typed or voice) into a validated intent:

    {"action":  "sort|count|status|stop|start|home|chat",
     "target":  "Apple|Orange|all|null",
     "segment": "S1|S2|S3|S4|null",     # sort a specific spot
     "count":   <int|null>,             # sort only N pieces
     "reply":   "short spoken confirmation"}

Safety model: the LLM only produces an intent; app.py validates everything
against a whitelist before anything reaches the arm.

ONLINE PROVIDERS (pick one — see AI_PROVIDERS.md for free options):
    set AI_PROVIDER=claude      + ANTHROPIC_API_KEY   (paid, best quality)
    set AI_PROVIDER=gemini      + GEMINI_API_KEY      (FREE tier)
    set AI_PROVIDER=groq        + GROQ_API_KEY        (FREE tier, fastest)
    set AI_PROVIDER=openrouter  + OPENROUTER_API_KEY  (FREE models)

No key / no internet -> a keyword fallback keeps basic commands working
("offline mode" tag in the chat).
"""
import json
import logging
import os
import re

import requests

logger = logging.getLogger(__name__)

VALID_ACTIONS = {"sort", "count", "status", "stop", "start", "home", "clear", "chat"}
VALID_TARGETS = {"Apple", "Orange", "all", None}
VALID_SEGMENTS = {"S1", "S2", "S3", "S4", None}

PROVIDERS = {
    "claude": {
        "env": "ANTHROPIC_API_KEY",
        "model": "claude-haiku-4-5-20251001",
    },
    "gemini": {
        "env": "GEMINI_API_KEY",
        "model": "gemini-2.5-flash",
        "url": "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions",
    },
    "groq": {
        "env": "GROQ_API_KEY",
        "model": "llama-3.3-70b-versatile",
        "url": "https://api.groq.com/openai/v1/chat/completions",
    },
    "openrouter": {
        "env": "OPENROUTER_API_KEY",
        "model": "meta-llama/llama-3.3-70b-instruct:free",
        "url": "https://openrouter.ai/api/v1/chat/completions",
    },
}

SYSTEM_PROMPT = """You are the voice assistant of an autonomous fruit-sorting robot.
A camera watches a 2x2 platform (segments S1 top-left, S2 top-right, S3
bottom-left, S4 bottom-right), a YOLO model detects Apples and Oranges, and a
5-DOF arm picks fruit into Bin 1 (apples) or Bin 2 (oranges).

You receive the user message plus a CONTEXT block with live detection data.
Respond with ONLY a JSON object, no markdown, no preamble:

{"action": "<sort|count|status|stop|start|home|clear|chat>",
 "target": "<Apple|Orange|all|null>",
 "segment": "<S1|S2|S3|S4|null>",
 "count": <integer or null>,
 "reply": "<short natural spoken reply, max 2 sentences>"}

Rules:
- "sort": user wants sorting. target = which fruit ("all" if unspecified).
  segment = only if they name a specific spot (e.g. "the apple on S1").
  count = only if they limit quantity (e.g. "sort 1 apple" -> 1).
  If they name a segment, check CONTEXT: if that segment is empty or holds a
  different fruit, say so in reply but still return the intent as asked.
- "count"/"status": answer from CONTEXT data in reply. No invented numbers.
- "stop" halts the arm immediately and pauses sorting.
- "clear" cancels everything: clears the command queue, forgets pending
  sector orders, stops sorting. Use for "clear the queue", "cancel all",
  "reset the orders".
- "start" resumes full autosort. "home" parks the arm.
- "chat": greetings/questions/anything not an actionable command. Politely
  decline anything unrelated or unsafe.
"""


def _fallback_intent(message: str, context: dict) -> dict:
    """Keyword parser used when no online provider is available."""
    m = message.lower()
    target = None
    if "apple" in m:
        target = "Apple"
    elif "orange" in m:
        target = "Orange"
    elif any(w in m for w in ("everything", "all", "both")):
        target = "all"

    seg = None
    seg_match = re.search(r"\bs\s?([1-4])\b", m)
    if seg_match:
        seg = "S" + seg_match.group(1)

    count = None
    m_no_seg = re.sub(r"\bs\s?[1-4]\b", " ", m)   # don't read "S1" as count 1
    num_match = re.search(r"\b(\d+)\b", m_no_seg)
    if num_match:
        count = int(num_match.group(1))
    for word, val in (("one", 1), ("two", 2), ("three", 3), ("four", 4)):
        if re.search(rf"\b{word}\b", m):
            count = val

    counts = context.get("platform_counts", {})
    a, o = counts.get("Apple", 0), counts.get("Orange", 0)

    if any(w in m for w in ("clear", "cancel", "flush", "reset queue", "reset the queue")):
        return {"action": "clear", "target": None, "segment": None, "count": None,
                "reply": "Cleared the queue and stopped sorting."}
    if any(w in m for w in ("stop", "halt", "pause", "freeze")):
        return {"action": "stop", "target": None, "segment": None, "count": None,
                "reply": "Stopping the arm."}
    if any(w in m for w in ("home", "park", "reset arm")):
        return {"action": "home", "target": None, "segment": None, "count": None,
                "reply": "Sending the arm home."}
    if any(w in m for w in ("sort", "put away", "clean", "pick", "grab")):
        t = target or "all"
        bits = []
        if count:
            bits.append(str(count))
        bits.append("everything" if t == "all" else t.lower() + ("s" if (count or 2) != 1 else ""))
        if seg:
            bits.append("on " + seg)
        return {"action": "sort", "target": t, "segment": seg, "count": count,
                "reply": "Sorting " + " ".join(bits) + "."}
    if any(w in m for w in ("start", "begin", "resume", "go")):
        return {"action": "start", "target": "all", "segment": None, "count": None,
                "reply": "Starting autosort."}
    if any(w in m for w in ("how many", "count", "what do you see", "what's on")):
        return {"action": "count", "target": target, "segment": None, "count": None,
                "reply": f"I can see {a} apple{'s' * (a != 1)} and {o} orange{'s' * (o != 1)} on the platform."}
    if "status" in m or "doing" in m:
        return {"action": "status", "target": None, "segment": None, "count": None,
                "reply": f"Autosort is {'running' if context.get('running') else 'paused'}; "
                         f"{a + o} fruit on the platform."}
    return {"action": "chat", "target": None, "segment": None, "count": None,
            "reply": "I can sort apples and oranges (even a specific spot like S1), "
                     "count what's on the platform, or stop and start the arm."}


def _extract_json(text: str):
    text = re.sub(r"```(?:json)?", "", text).strip()
    start, end = text.find("{"), text.rfind("}")
    if start == -1 or end == -1:
        raise ValueError("no JSON object in response")
    return json.loads(text[start:end + 1])


def _validate(intent: dict) -> dict:
    action = intent.get("action")
    if action not in VALID_ACTIONS:
        action = "chat"
    target = intent.get("target")
    if target in ("null", "", "None"):
        target = None
    if target not in VALID_TARGETS:
        target = None
    segment = intent.get("segment")
    if segment in ("null", "", "None"):
        segment = None
    if segment not in VALID_SEGMENTS:
        segment = None
    count = intent.get("count")
    if not isinstance(count, int) or count <= 0 or count > 20:
        count = None
    reply = str(intent.get("reply") or "Okay.")[:300]
    return {"action": action, "target": target, "segment": segment,
            "count": count, "reply": reply}


def _call_claude(api_key: str, model: str, message: str, context: dict) -> str:
    resp = requests.post(
        "https://api.anthropic.com/v1/messages",
        headers={"x-api-key": api_key, "anthropic-version": "2023-06-01",
                 "content-type": "application/json"},
        json={"model": model, "max_tokens": 300, "system": SYSTEM_PROMPT,
              "messages": [{"role": "user",
                            "content": f"CONTEXT:\n{json.dumps(context)}\n\nUSER MESSAGE:\n{message}"}]},
        timeout=12)
    resp.raise_for_status()
    data = resp.json()
    return "".join(b.get("text", "") for b in data.get("content", [])
                   if b.get("type") == "text")


def _call_openai_style(url: str, api_key: str, model: str,
                       message: str, context: dict) -> str:
    """Gemini (OpenAI-compat endpoint), Groq, OpenRouter."""
    resp = requests.post(
        url,
        headers={"Authorization": f"Bearer {api_key}",
                 "Content-Type": "application/json"},
        json={"model": model,
              "messages": [
                  {"role": "system", "content": SYSTEM_PROMPT},
                  {"role": "user",
                   "content": f"CONTEXT:\n{json.dumps(context)}\n\nUSER MESSAGE:\n{message}"},
              ],
              "max_tokens": 300},
        timeout=15)
    resp.raise_for_status()
    return resp.json()["choices"][0]["message"]["content"]


def parse_intent(message: str, context: dict) -> dict:
    provider = os.environ.get("AI_PROVIDER", "claude").strip().lower()
    cfg = PROVIDERS.get(provider, PROVIDERS["claude"])
    api_key = os.environ.get(cfg["env"], "").strip()

    if not api_key:
        logger.warning("%s not set — using offline fallback parser", cfg["env"])
        out = _fallback_intent(message, context)
        out["engine"] = "offline"
        return out

    try:
        if provider == "claude" or "url" not in cfg:
            text = _call_claude(api_key, cfg["model"], message, context)
        else:
            text = _call_openai_style(cfg["url"], api_key, cfg["model"],
                                      message, context)
        out = _validate(_extract_json(text))
        out["engine"] = provider
        return out
    except Exception as e:                                    # noqa: BLE001
        logger.error("%s API failed (%s) — falling back offline", provider, e)
        out = _fallback_intent(message, context)
        out["engine"] = "offline"
        return out


if __name__ == "__main__":
    ctx = {"running": False, "arm_busy": False,
           "platform_counts": {"Apple": 2, "Orange": 1},
           "segments": {"S1": "Apple", "S2": "Apple", "S3": "Orange", "S4": None},
           "sorted_totals": {"Apple": 4, "Orange": 6}}
    for msg in ["sort 1 apple on S1", "sort the oranges", "grab two apples",
                "how many apples do you see?", "stop"]:
        print(msg, "->", parse_intent(msg, ctx))
