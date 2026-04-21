const statusList = document.getElementById("status-list");
const logSelect = document.getElementById("log-select");
const logView = document.getElementById("log-view");
const toast = document.getElementById("toast");
const nodeList = document.getElementById("node-list");
const actionList = document.getElementById("action-list");
const topicList = document.getElementById("topic-list");

let selectedLog = "";
const ACTION_LABELS = {
  build_workspace: "Prepare ROS2 Runtime",
  ros2_server: "Start ROS2 Server",
  ros2_send_goal: "Send Pick-and-Place Goal",
};

function setInputValue(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  if (Array.isArray(value)) {
    element.value = value.join(",");
  } else {
    element.value = String(value ?? "");
  }
}

function parseNumberList(value) {
  return value
    .split(",")
    .map((item) => item.trim())
    .filter(Boolean)
    .map((item) => Number(item));
}

function showToast(message, isError = false) {
  toast.textContent = message;
  toast.classList.remove("hidden");
  toast.style.background = isError ? "#8c2f39" : "#17594a";
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => toast.classList.add("hidden"), 2600);
}

async function postJson(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    throw new Error(`Request failed: ${response.status}`);
  }
  return response.json();
}

async function runAction(action) {
  const payload = { action };
  if (action === "ros2_server") {
    payload.base_url = document.getElementById("base-url").value.trim();
    payload.safe_height_mm = Number(document.getElementById("safe-height-mm").value);
    payload.waypoint_delay_ms = Number(document.getElementById("waypoint-delay-ms").value);
    payload.gripper_close_angle = Number(document.getElementById("gripper-close-angle").value);
    payload.steps_per_segment = Number(document.getElementById("steps-per-segment").value);
    payload.arrival_tolerance_mm = Number(document.getElementById("arrival-tolerance-mm").value);
  }
  if (action === "ros2_send_goal") {
    payload.pick_position = parseNumberList(document.getElementById("pick-position").value);
    payload.drop_position = parseNumberList(document.getElementById("drop-position").value);
  }

  try {
    await postJson("/api/run", payload);
    showToast(`Started: ${action}`);
    refreshAll();
  } catch (error) {
    showToast(error.message, true);
  }
}

async function stopProcess(key) {
  try {
    await postJson("/api/stop", { key });
    showToast(`Stopped: ${key}`);
    refreshAll();
  } catch (error) {
    showToast(error.message, true);
  }
}

async function refreshStatus() {
  const response = await fetch("/api/status");
  const data = await response.json();
  renderStatus(data.processes);
  renderLogOptions(data.processes);
}

function renderStatus(processes) {
  if (!processes.length) {
    statusList.innerHTML = "<div class='status-item'>No dashboard actions have been started yet.</div>";
    return;
  }

  statusList.innerHTML = "";
  for (const process of processes) {
    const item = document.createElement("div");
    item.className = "status-item";
    item.innerHTML = `
      <div class="status-top">
        <div class="status-name">${process.label}</div>
        <span class="badge ${process.status}">${process.status}</span>
      </div>
      <div class="status-meta">key: ${process.key}</div>
      <div class="status-meta">exit code: ${process.exit_code ?? "n/a"}</div>
    `;
    if (process.long_running && process.status === "running") {
      const stopButton = document.createElement("button");
      stopButton.className = "secondary small";
      stopButton.textContent = "Stop";
      stopButton.addEventListener("click", () => stopProcess(process.key));
      item.appendChild(stopButton);
    }
    statusList.appendChild(item);
  }
}

function choosePreferredLog(processes, currentSelection) {
  const keys = processes.map((item) => item.key);
  if (!keys.length) {
    return "";
  }

  const preferredOrder = [
    { key: "ros2_send_goal", statuses: ["running", "finished", "failed"] },
    { key: "ros2_server", statuses: ["running", "finished", "failed"] },
    { key: "build_workspace", statuses: ["running", "finished", "failed"] },
  ];

  for (const preferred of preferredOrder) {
    const match = processes.find(
      (item) => item.key === preferred.key && preferred.statuses.includes(item.status)
    );
    if (match) {
      return match.key;
    }
  }

  if (currentSelection && keys.includes(currentSelection)) {
    return currentSelection;
  }

  return keys[keys.length - 1];
}

function renderLogOptions(processes) {
  const keys = processes.map((item) => item.key);
  if (!keys.length) {
    logSelect.innerHTML = "";
    logView.textContent = "No process selected yet.";
    selectedLog = "";
    return;
  }

  selectedLog = choosePreferredLog(processes, selectedLog);

  logSelect.innerHTML = "";
  for (const process of processes) {
    const option = document.createElement("option");
    option.value = process.key;
    option.textContent = `${ACTION_LABELS[process.key] || process.label} (${process.status})`;
    option.selected = process.key === selectedLog;
    logSelect.appendChild(option);
  }
}

async function refreshLog() {
  if (!selectedLog) {
    return;
  }
  const wasNearBottom =
    (logView.scrollHeight - logView.scrollTop - logView.clientHeight) < 24;
  const response = await fetch(`/api/logs/${selectedLog}`);
  const data = await response.json();
  logView.textContent = data.log || "No log output yet.";
  if (wasNearBottom) {
    logView.scrollTop = logView.scrollHeight;
  }
}

async function refreshRos2State() {
  const response = await fetch("/api/ros2_state");
  const data = await response.json();
  renderList(nodeList, data.nodes, "No ROS2 nodes visible yet.");
  renderList(actionList, data.actions, "No ROS2 actions visible yet.");
  renderList(topicList, data.topics, "No ROS2 topics visible yet.");
}

function renderList(element, values, emptyText) {
  element.innerHTML = "";
  if (!values.length) {
    element.innerHTML = `<li>${emptyText}</li>`;
    return;
  }
  for (const value of values) {
    const item = document.createElement("li");
    item.textContent = value;
    element.appendChild(item);
  }
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function renderEvidencePanels(target, panels, emptyTitle, emptyNote) {
  if (!target) {
    return;
  }
  target.innerHTML = "";
  if (!panels.length) {
    target.innerHTML = `
      <div class="evidence-item">
        <div class="evidence-title">${escapeHtml(emptyTitle)}</div>
        <div class="evidence-subtitle">${escapeHtml(emptyNote)}</div>
      </div>
    `;
    return;
  }

  for (const panel of panels) {
    const item = document.createElement("div");
    item.className = "evidence-item";
    const subtitle = panel.subtitle ? `<div class="evidence-subtitle">${escapeHtml(panel.subtitle)}</div>` : "";
    const code = panel.code ? `<pre class="evidence-code">${escapeHtml(panel.code)}</pre>` : "";
    item.innerHTML = `
      <div class="evidence-title">${escapeHtml(panel.title || "Evidence")}</div>
      ${subtitle}
      ${code}
    `;
    target.appendChild(item);
  }
}

async function refreshRos2Summary() {
  const response = await fetch("/api/ros2_summary");
  const data = await response.json();

  void data;
}

async function loadDefaults() {
  const response = await fetch("/api/defaults");
  const data = await response.json();
  setInputValue("base-url", data.base_url);
  setInputValue("safe-height-mm", data.safe_height_mm);
  setInputValue("waypoint-delay-ms", data.waypoint_delay_ms);
  setInputValue("gripper-close-angle", data.gripper_close_angle);
  setInputValue("steps-per-segment", data.steps_per_segment);
  setInputValue("arrival-tolerance-mm", data.arrival_tolerance_mm);
  setInputValue("pick-position", data.pick_position);
  setInputValue("drop-position", data.drop_position);
}

async function refreshAll() {
  await Promise.allSettled([refreshStatus(), refreshLog(), refreshRos2State(), refreshRos2Summary()]);
}

document.querySelectorAll("[data-action]").forEach((button) => {
  button.addEventListener("click", () => runAction(button.dataset.action));
});

document.getElementById("stop-all").addEventListener("click", async () => {
  try {
    await postJson("/api/stop_all", {});
    showToast("Stopped all dashboard processes.");
    refreshAll();
  } catch (error) {
    showToast(error.message, true);
  }
});

document.getElementById("clear-finished").addEventListener("click", async () => {
  try {
    await postJson("/api/clear_finished", {});
    showToast("Cleared finished and failed entries.");
    refreshAll();
  } catch (error) {
    showToast(error.message, true);
  }
});

document.getElementById("refresh-status").addEventListener("click", refreshAll);

logSelect.addEventListener("change", () => {
  selectedLog = logSelect.value;
  refreshLog();
});

window.setInterval(() => {
  refreshAll().catch(() => {});
}, 1800);

Promise.allSettled([loadDefaults(), refreshAll()]).catch(() => {});
