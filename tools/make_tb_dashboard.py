#!/usr/bin/env python3
"""
make_tb_dashboard.py - generate the ThingsBoard dashboard for this gateway.

WHY A GENERATOR AND NOT A HAND-WRITTEN JSON
    A ThingsBoard widget's configuration is large, version-specific and mostly
    boilerplate. Writing one by hand means inventing fields that the running
    server may not accept, and the failure mode is nasty: the dashboard imports
    without complaint and then renders wrong.

    So this script fetches each widget type from ThingsBoard's own source, takes
    its published defaultConfig, and overrides only the handful of fields that
    are actually about this gateway - the datasource, the RPC method names, the
    title. Everything else is whatever ThingsBoard itself would have used when
    you added the widget by hand.

    It also means the dashboard can be regenerated for a different ThingsBoard
    version by pointing --ref at that tag.

USAGE
    python tools/make_tb_dashboard.py
    python tools/make_tb_dashboard.py --ref release-3.7.0 --out dashboards/gw.json
    python tools/make_tb_dashboard.py --device-name plant-a-gw

    Then in ThingsBoard: Dashboards -> "+" -> Import dashboard.

    Needs network access to raw.githubusercontent.com. The generated file is
    committed, so this only has to run when the dashboard changes.

WHAT IT BUILDS
    4 switch controls   relay 1..4, RPC setRelay1..4 / getRelay1..4
    4 value cards       input 1..4, from telemetry
    2 timeseries tables I/O history, and link diagnostics

HOW THE WIDGETS FIND THE DEVICE
    By device name. --device-name (default "IoTPLC") must match the device's
    name in ThingsBoard exactly - the name shown in the device list, not the
    label and not the access token.

    ThingsBoard always resolves a widget's data through an entity alias, so one
    is still emitted, but it is a thin wrapper: named after the device and
    filtering on that name, nothing more. Nothing to pick from a dropdown after
    importing.

    A name filter rather than a device id, because a device id only exists on
    the instance that created it - a committed file carrying one resolves to
    nothing on any other server, and does it silently.
"""

import argparse
import json
import os
import random
import sys
import tempfile
import urllib.request
import uuid

RAW = "https://raw.githubusercontent.com/thingsboard/thingsboard/{ref}/application/src/main/data/json/system/widget_types/{name}.json"

RELAY_COUNT = 4
INPUT_COUNT = 4

# Grid is 24 columns wide, which is the ThingsBoard default.
GRID_COLUMNS = 24


# Namespace for the generated ids below. Any fixed UUID does; this one is
# arbitrary and must simply never change.
ID_NAMESPACE = uuid.UUID("6f9b1f2e-49a2-5c31-9b3f-1c0a7d5e4b88")


def stable_id(role):
    """
    A UUID derived from a role name rather than a fresh random one.

    Regenerating this dashboard should produce the same file, not a file where
    every id has moved. Random ids would make every regeneration a whole-file
    diff with nothing readable in it, and would turn a re-import into a second
    copy of every widget instead of an update of the first.
    """
    return str(uuid.uuid5(ID_NAMESPACE, "gw-dashboard:" + role))


def fetch_widget(ref, name, cache_dir):
    """Returns (fqn, descriptor) for one system widget type, with a disk cache."""
    cached = os.path.join(cache_dir, "%s@%s.json" % (name, ref.replace("/", "_")))
    if os.path.exists(cached):
        with open(cached, "r", encoding="utf-8") as fh:
            return json.load(fh)

    url = RAW.format(ref=ref, name=name)
    sys.stderr.write("fetching %s\n" % url)
    with urllib.request.urlopen(url, timeout=30) as resp:
        data = json.loads(resp.read().decode("utf-8"))

    os.makedirs(cache_dir, exist_ok=True)
    with open(cached, "w", encoding="utf-8") as fh:
        json.dump(data, fh)
    return data


def default_config(widget):
    cfg = widget["descriptor"].get("defaultConfig")
    if isinstance(cfg, str):
        cfg = json.loads(cfg)
    cfg = json.loads(json.dumps(cfg))  # deep copy

    # Drop configMode.
    #
    # Newer widgets ship configMode "basic", which puts ThingsBoard's simplified
    # editor in charge of the binding. Everything here is written the advanced
    # way - an explicit datasources array - and the two disagree: the widget
    # looks for its value where the basic editor would have put it, finds
    # nothing, and renders a card that never changes. ThingsBoard's own
    # dashboards omit the key entirely on widgets configured this way.
    cfg.pop("configMode", None)
    return cfg


def constant_colors(node):
    """
    Rewrites ThingsBoard's demo colour functions to plain constants.

    The stock value card ships a colourFunction written for a temperature demo -
    it mixes blue to red across -60..60. Left in place on a boolean it produces
    colours that look meaningful and are not, which is worse than no colour at
    all.
    """
    if isinstance(node, dict):
        if node.get("type") in ("range", "function") and "color" in node:
            node["type"] = "constant"
            node.pop("colorFunction", None)
            node.pop("rangeList", None)
        for value in node.values():
            constant_colors(value)
    elif isinstance(node, list):
        for value in node:
            constant_colors(value)
    return node


def data_key(name, label, color="#2196f3", decimals=0, key_type="timeseries",
             boolean=False):
    """
    One data key.

    _hash is what ThingsBoard uses to tell the keys of a datasource apart. Its
    own dashboards carry a distinct random float per key; an earlier version of
    this generator emitted 0.0 for every key, which collides as soon as a
    datasource has more than one - and the symptom is a column that never
    updates rather than an error. Seeding from the key name keeps it unique per
    key and stable across regenerations. Together with stable_id() above, that
    is what makes regenerating this dashboard a no-op when nothing changed.

    `boolean` switches on the cell renderer ThingsBoard uses for booleans in its
    own demo dashboard: a filled circle whose colour follows the value. Note the
    comparison is against the STRING "true" - the table hands the style function
    a formatted cell value, not the raw JSON boolean, and comparing against a
    real `true` silently never matches.
    """
    settings = {}
    if boolean:
        settings = {
            "columnWidth": "0px",
            "useCellStyleFunction": True,
            "useCellContentFunction": True,
            "cellContentFunction": "return '&#11044;';",
            "cellStyleFunction": (
                "var on = (value === true || value === 'true' || value === 1 "
                "|| value === '1');\n"
                "return {\n"
                "    color: on ? 'rgb(39, 134, 34)' : 'rgba(0, 0, 0, 0.26)',\n"
                "    fontSize: '22px'\n"
                "};"
            ),
        }
    return {
        "name": name,
        "type": key_type,
        "label": label,
        "color": color,
        "settings": settings,
        "decimals": decimals,
        "funcBody": None,
        "aggregationType": None,
        "units": None,
        "_hash": stable_hash(name),
    }


def stable_hash(name):
    """A per-key float in [0,1), derived from the key name so it never changes."""
    return random.Random("gw-datakey:" + name).random()


def entity_datasource(alias_id, keys):
    return [{
        "type": "entity",
        "name": "",
        "entityAliasId": alias_id,
        "filterId": None,
        "dataKeys": keys,
    }]


def build(ref, cache_dir, device_name, title):
    alias_id = stable_id("alias:" + device_name)

    switch = fetch_widget(ref, "switch_control", cache_dir)
    entities_table = fetch_widget(ref, "entities_table", cache_dir)
    ts_table = fetch_widget(ref, "timeseries_table", cache_dir)

    widgets = {}
    layout = {}

    def place(widget_id, col, row, size_x, size_y):
        layout[widget_id] = {"sizeX": size_x, "sizeY": size_y, "row": row, "col": col,
                             "mobileHeight": None, "mobileOrder": None}

    # ---- relays: one switch control each -----------------------------------
    for i in range(1, RELAY_COUNT + 1):
        wid = stable_id("relay%d" % i)
        cfg = default_config(switch)

        cfg["title"] = "Relay %d" % i
        cfg["showTitle"] = True
        # Both spellings of the target device. Older ThingsBoard reads
        # targetDeviceAliases, newer reads targetDevice; a server ignores the
        # one it does not know, so carrying both makes the file import cleanly
        # across versions instead of silently losing its device binding.
        cfg["targetDeviceAliases"] = [alias_id]
        cfg["targetDevice"] = {"type": "entity", "entityAliasId": alias_id}

        cfg["settings"].update({
            "title": "Relay %d" % i,
            "getValueMethod": "getRelay%d" % i,
            "setValueMethod": "setRelay%d" % i,
            "initialValue": False,
            "showOnOffLabels": True,
            # The firmware confirms a relay by re-reading the pins before it
            # answers, so the round trip is a link transaction plus MQTT rather
            # than a local variable. 500 ms is the stock value and is too tight
            # for that on a busy network.
            "requestTimeout": 5000,
        })

        widgets[wid] = {
            "id": wid,
            "typeFullFqn": "system." + switch["fqn"],
            "type": switch["descriptor"]["type"],
            "sizeX": 6,
            "sizeY": 3,
            "config": cfg,
            "row": 0,
            "col": 0,
        }
        place(wid, (i - 1) * 6, 0, 6, 3)

    # ---- inputs: one live table of booleans ---------------------------------
    #
    # NOT four value cards. A value card is a numeric readout - 52px digits,
    # decimals, units - and these inputs are booleans. ThingsBoard renders a
    # boolean in a table, using the per-column cell functions that its own demo
    # dashboard uses for exactly this, so that is what they get: one row, four
    # lamps that are green when the contact is closed.
    wid = stable_id("inputs")
    cfg = default_config(entities_table)

    cfg["title"] = "Digital inputs - PD4..PD7"
    cfg["showTitle"] = True
    cfg["datasources"] = entity_datasource(alias_id, [
        data_key("input%d" % i, "Input %d" % i, "#4caf50", boolean=True)
        for i in range(1, INPUT_COUNT + 1)
    ])
    if isinstance(cfg.get("settings"), dict):
        cfg["settings"].update({
            "entitiesTitle": "Device",
            "displayEntityName": True,
            "displayEntityType": False,
            "displayEntityLabel": False,
            # One device, one row. Pagination and search on a single row is
            # furniture that costs half the widget's height.
            "displayPagination": False,
            "enableSearch": False,
            "defaultPageSize": 10,
        })
    constant_colors(cfg.get("settings", {}))

    widgets[wid] = {
        "id": wid,
        "typeFullFqn": "system." + entities_table["fqn"],
        "type": entities_table["descriptor"]["type"],
        "sizeX": 24,
        "sizeY": 4,
        "config": cfg,
        "row": 0,
        "col": 0,
    }
    place(wid, 0, 3, 24, 4)

    # ---- history table ------------------------------------------------------
    wid = stable_id("io-history")
    cfg = default_config(ts_table)
    cfg["title"] = "I/O history"
    cfg["showTitle"] = True
    keys = [data_key("relay%d" % i, "Relay %d" % i, "#f44336", boolean=True)
            for i in range(1, RELAY_COUNT + 1)]
    keys += [data_key("input%d" % i, "Input %d" % i, "#4caf50", boolean=True)
             for i in range(1, INPUT_COUNT + 1)]
    cfg["datasources"] = entity_datasource(alias_id, keys)
    constant_colors(cfg.get("settings", {}))
    widgets[wid] = {
        "id": wid,
        "typeFullFqn": "system." + ts_table["fqn"],
        "type": ts_table["descriptor"]["type"],
        "sizeX": 12,
        "sizeY": 7,
        "config": cfg,
        "row": 0,
        "col": 0,
    }
    place(wid, 0, 7, 12, 7)

    # ---- diagnostics table --------------------------------------------------
    wid = stable_id("diagnostics")
    cfg = default_config(ts_table)
    cfg["title"] = "Link and gateway health"
    cfg["showTitle"] = True
    cfg["datasources"] = entity_datasource(alias_id, [
        # ioQuality first: it is the one column that says whether any of the
        # others can be believed.
        data_key("ioQuality", "IO quality", "#9c27b0"),
        data_key("linkRttUs", "Link RTT (us)", "#2196f3"),
        data_key("linkTimeouts", "Link timeouts", "#ff9800"),
        data_key("linkCrcErrors", "Link CRC errors", "#f44336"),
        data_key("rssi", "WiFi RSSI (dBm)", "#607d8b"),
        data_key("freeHeap", "Free heap", "#795548"),
        data_key("uptimeSec", "Uptime (s)", "#009688"),
    ])
    constant_colors(cfg.get("settings", {}))
    widgets[wid] = {
        "id": wid,
        "typeFullFqn": "system." + ts_table["fqn"],
        "type": ts_table["descriptor"]["type"],
        "sizeX": 12,
        "sizeY": 7,
        "config": cfg,
        "row": 0,
        "col": 0,
    }
    place(wid, 12, 7, 12, 7)

    dashboard = {
        "title": title,
        "name": title,
        "image": None,
        "mobileHide": False,
        "mobileOrder": None,
        # Present and empty, the way an exported dashboard carries it.
        "resources": [],
        "configuration": {
            "description": (
                "STM32F407 + ESP32-S3 IIoT gateway, device '%s'. Four relays on "
                "STM32 PD0..PD3 (active low, switched by RPC) and four pulled-up "
                "inputs on PD4..PD7. Generated by tools/make_tb_dashboard.py."
                % device_name
            ),
            "widgets": widgets,
            "states": {
                "default": {
                    "name": title,
                    "root": True,
                    "layouts": {
                        "main": {
                            "widgets": layout,
                            "gridSettings": {
                                "backgroundColor": "#eeeeee",
                                "columns": GRID_COLUMNS,
                                "margin": 10,
                                "backgroundSizeMode": "100%",
                                "autoFillHeight": False,
                                "mobileAutoFillHeight": False,
                                "mobileRowHeight": 70,
                            },
                        }
                    },
                }
            },
            "entityAliases": {
                alias_id: {
                    "id": alias_id,
                    # Named after the device, so the alias reads as the
                    # device rather than as an indirection to look up.
                    "alias": device_name,
                    # Resolved by device name, not by id. A device id only
                    # exists on the instance that created it, so a committed
                    # file carrying one resolves to nothing elsewhere - and
                    # does it silently, which is the worst way to fail.
                    "filter": {
                        "type": "entityName",
                        "resolveMultiple": False,
                        "entityType": "DEVICE",
                        "entityNameFilter": device_name,
                    },
                }
            },
            "filters": {},
            "timewindow": {
                "displayValue": "",
                "selectedTab": 0,
                "realtime": {"interval": 1000, "timewindowMs": 900000},
                "aggregation": {"type": "NONE", "limit": 25000},
            },
            "settings": {
                "stateControllerId": "entity",
                "showTitle": True,
                "showDashboardsSelect": True,
                "showEntitiesSelect": True,
                "showDashboardTimewindow": True,
                "showDashboardExport": True,
                "toolbarAlwaysOpen": True,
            },
        },
    }
    return dashboard


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)

    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", default="master",
                    help="ThingsBoard git ref to take widget defaults from (default: master)")
    ap.add_argument("--out", default=os.path.join(repo, "dashboards",
                                                  "iiot_gateway_dashboard.json"))
    # Cache outside the repository. This one has no .gitignore, and a tool that
    # drops downloaded files into the tree adds noise to every later diff.
    ap.add_argument("--cache",
                    default=os.path.join(tempfile.gettempdir(), "tb_widget_cache"))
    ap.add_argument("--device-name", default="IoTPLC",
                    help="ThingsBoard device name the widgets bind to "
                         "(default: IoTPLC)")
    ap.add_argument("--title", default="IoTPLC - relays and inputs")
    args = ap.parse_args()

    dashboard = build(args.ref, args.cache, args.device_name, args.title)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(dashboard, fh, indent=2)
        fh.write("\n")

    widgets = dashboard["configuration"]["widgets"]
    print("wrote %s" % args.out)
    print("  %d widgets, all bound to device name '%s'"
          % (len(widgets), args.device_name))
    for w in widgets.values():
        print("    %-40s %s" % (w["config"].get("title", ""), w["typeFullFqn"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
