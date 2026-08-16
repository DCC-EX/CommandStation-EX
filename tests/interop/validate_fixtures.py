"""Validate the CommandStation-side interoperability contract."""
import json
from pathlib import Path

REQUIRED = {"parsing", "power", "accessories", "programming", "heartbeat"}

def validate(scenarios):
    assert isinstance(scenarios, list) and scenarios
    ids, capabilities = set(), set()
    for scenario in scenarios:
        assert scenario["id"] not in ids, scenario["id"]
        ids.add(scenario["id"])
        capabilities.add(scenario["capability"])
        assert scenario["clients"] and scenario["steps"]
        for step in scenario["steps"]:
            assert step.get("client") in scenario["clients"]
            assert "input" in step or "event" in step
            if "input" in step:
                assert step["input"].startswith("<")
                assert all(reply.startswith("<") for reply in step["replies"])
            for broadcast in step.get("broadcasts", []):
                target = broadcast["broadcast_to"]
                assert target == "all" or target == step["client"] or set(target) <= set(scenario["clients"])
                assert broadcast["message"].startswith("<")
        if scenario["capability"] == "heartbeat":
            assert any(step.get("heartbeat") for step in scenario["steps"])
            assert any(step.get("event") == "disconnect" for step in scenario["steps"])
            assert any(step.get("event") == "connect" for step in scenario["steps"])
    assert REQUIRED <= capabilities, sorted(REQUIRED - capabilities)
    assert any(len(s["clients"]) > 1 for s in scenarios)

if __name__ == "__main__":
    scenarios = json.loads(Path(__file__).with_name("scenarios.json").read_text(encoding="utf-8"))
    validate(scenarios)
    print(f"validated {len(scenarios)} interoperability scenarios")
