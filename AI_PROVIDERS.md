# AI Providers — getting the assistant ONLINE (including FREE options)

## Why you're seeing "offline mode"

The assistant tags replies **offline mode** when no AI API key is set. It
falls back to a built-in keyword parser — commands still work, but it can't
handle free-form phrasing or answer open questions. To go online, pick ONE
provider below, get its key, and set two environment variables before
running app.py.

The keys go in the SAME PowerShell window you launch from (they last until
the window closes):

```
$env:AI_PROVIDER = "<provider>"
$env:<KEY_NAME>  = "<your key>"
python app.py
```

To set them permanently (all future windows):
```
setx AI_PROVIDER "<provider>"
setx <KEY_NAME> "<your key>"
```
(then open a NEW terminal)

---

## Option 1 — Google Gemini  (FREE, recommended free pick)

- Free tier with a daily request allowance on Flash models — far more than a
  demo day needs. No credit card required.
- Get a key: https://aistudio.google.com → sign in → **Get API key** →
  Create API key.

```
$env:AI_PROVIDER = "gemini"
$env:GEMINI_API_KEY = "AIza..."
```
Model used: `gemini-2.5-flash`.

## Option 2 — Groq  (FREE, fastest responses)

- Free tier running open models (Llama 3.3 70B) on very fast hardware —
  replies feel instant, great for voice.
- Get a key: https://console.groq.com → sign in → **API Keys** → Create.

```
$env:AI_PROVIDER = "groq"
$env:GROQ_API_KEY = "gsk_..."
```
Model used: `llama-3.3-70b-versatile`.

## Option 3 — OpenRouter  (FREE models)

- A router to many models; the ones tagged `:free` cost nothing (daily
  limits apply).
- Get a key: https://openrouter.ai → sign in → **Keys** → Create.

```
$env:AI_PROVIDER = "openrouter"
$env:OPENROUTER_API_KEY = "sk-or-..."
```
Model used: `meta-llama/llama-3.3-70b-instruct:free`.

## Option 4 — Anthropic Claude  (paid, best quality)

- The default provider. Each parsed command costs a fraction of a centavo
  on the Haiku model, but the account needs credits.
- Get a key: https://console.anthropic.com → **API Keys**.

```
$env:AI_PROVIDER = "claude"
$env:ANTHROPIC_API_KEY = "sk-ant-..."
```
Model used: `claude-haiku-4-5-20251001`.

---

## How to verify it's working

1. Launch app.py and send the assistant: `sort 1 apple on S1`.
2. An **online** reply has no "offline mode" tag under the bubble.
3. The terminal running app.py logs which engine parsed each message.

Quick standalone test without the dashboard:
```
$env:AI_PROVIDER = "gemini"; $env:GEMINI_API_KEY = "AIza..."
python assistant.py
```
The printed intents should show `'engine': 'gemini'` (not `'offline'`).

## Notes & troubleshooting

- If the provider errors mid-demo (rate limit, WiFi drop), the app
  automatically falls back to the offline parser — voice control never
  fully dies. That's also your defense-day safety net.
- Free-tier rate limits (per-minute/per-day) are documented on each
  provider's console page and can change — check there if requests start
  failing. A sorting demo uses a handful of requests, so limits are rarely
  an issue.
- Keep keys out of the code and out of your repo/paper appendix. Anyone
  with the key can spend your quota.
- What the "assistant" actually is: an LLM that converts speech/text into a
  strict JSON intent (`sort/count/status/stop/start/home`) which app.py
  validates against a whitelist before anything reaches the arm. Whichever
  provider you pick, the arm can only ever receive the same six validated
  actions.
