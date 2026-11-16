import json, os, shutil, sqlite3, time

HOME = os.path.expanduser("~")
STATE = os.path.join(HOME, ".codex", ".codex-global-state.json")
DB = os.path.join(HOME, ".codex", "state_5.sqlite")
MODEL = "gpt-5.6-sol"
stamp = time.strftime("%Y%m%d-%H%M%S")

shutil.copy2(STATE, STATE + f".bak-{stamp}")
shutil.copy2(DB, DB + f".bak-{stamp}")
print("backups:", STATE + f".bak-{stamp}", "|", DB + f".bak-{stamp}")

data = json.loads(open(STATE, encoding="utf-8").read())
atom = data.setdefault("electron-persisted-atom-state", {})
atom["composer-recent-model-configurations-v1"] = [
    {"model": MODEL, "reasoningEffort": "medium", "serviceTier": None}
]
open(STATE, "w", encoding="utf-8").write(json.dumps(data, ensure_ascii=False))
print("global model selection ->", MODEL)

con = sqlite3.connect(DB)
cur = con.cursor()
cur.execute("UPDATE threads SET model = ? WHERE model IS NOT NULL AND model <> ?", (MODEL, MODEL))
print("threads updated:", cur.rowcount)
con.commit()
cur.execute("SELECT id, model FROM threads ORDER BY updated_at_ms DESC LIMIT 6")
for r in cur.fetchall():
    print("  ", r)
con.close()
