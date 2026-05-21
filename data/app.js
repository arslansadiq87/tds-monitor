(() => {
  const el = (id) => document.getElementById(id);

  const tdsEl = el("tdsValue");
  const voltEl = el("voltageValue");
  const ageEl = el("ageValue");
  const uptimeEl = el("uptimeValue");
  const apiEl = el("apiValue");
  const tempEl = el("tempValue");
  const waterLevelPercentEl = el("waterLevelPercent");
  const waterLevelLabelEl = el("waterLevelLabel");
  const waterLevelLitersEl = el("waterLevelLiters");
  const waterLevelBar = el("waterLevelBar");
  const waterHeightValueEl = el("waterHeightValue");
  const oddLevelValueEl = el("oddLevelValue");
  const evenLevelValueEl = el("evenLevelValue");
  const levelAgeValueEl = el("levelAgeValue");
  const qualityPill = el("qualityPill");
  const wifiChip = el("wifiChip");
  const wifiText = el("wifiText");
  const dateText = el("dateText");
  const refreshMsEl = el("refreshMs");
  const tempBar = el("tempBar");

  const btnRefresh = el("btnRefresh");
  const canvas = el("chart");
  const ctx = canvas.getContext("2d");

  const REFRESH_MS = 1000;
  refreshMsEl.textContent = String(REFRESH_MS);

  // Keep a small rolling buffer for the chart
  const maxPoints = 60;
  const series = [];

  function msToHuman(ms) {
    if (ms == null) return "--";
    const s = Math.floor(ms / 1000);
    if (s < 60) return `${s}s`;
    const m = Math.floor(s / 60);
    if (m < 60) return `${m}m ${s % 60}s`;
    const h = Math.floor(m / 60);
    return `${h}h ${m % 60}m`;
  }

  function quality(ppm) {
    // Simple drinking-water style bands (adjust as you like)
    if (ppm == null || Number.isNaN(ppm)) return { text: "--", cls: "" };
    if (ppm <= 300) return { text: "Good", cls: "good" };
    if (ppm <= 600) return { text: "Moderate", cls: "mid" };
    return { text: "High", cls: "bad" };
  }

  function formatLadder(name, sw, voltage) {
    const parts = [name];
    if (sw != null && Number(sw) > 0) parts.push(`SW${sw}`);
    else parts.push("open");
    if (voltage == null || Number.isNaN(voltage)) parts.push("-- V");
    else parts.push(`${voltage.toFixed(2)} V`);
    return parts.join(" • ");
  }

  function setPill(q) {
    qualityPill.textContent = q.text;
    qualityPill.style.borderColor =
      q.cls === "good" ? "rgba(50,255,170,.45)" :
      q.cls === "mid"  ? "rgba(64,170,255,.45)" :
      q.cls === "bad"  ? "rgba(255,90,120,.45)" : "rgba(255,255,255,.12)";
    qualityPill.style.boxShadow =
      q.cls === "good" ? "0 0 18px rgba(50,255,170,.12)" :
      q.cls === "mid"  ? "0 0 18px rgba(64,170,255,.12)" :
      q.cls === "bad"  ? "0 0 18px rgba(255,90,120,.10)" : "none";
  }

  function drawChart() {
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Grid
    ctx.globalAlpha = 0.35;
    ctx.strokeStyle = "white";
    ctx.lineWidth = 1;
    const steps = 6;
    for (let i = 1; i < steps; i++) {
      const y = (h * i) / steps;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

    if (series.length < 2) return;

    const ys = series.map(p => p.v);
    const min = Math.min(...ys);
    const max = Math.max(...ys);
    const pad = Math.max(5, (max - min) * 0.1);
    const lo = min - pad;
    const hi = max + pad;

    const xStep = w / (maxPoints - 1);
    const yMap = (v) => {
      if (hi === lo) return h / 2;
      return h - ((v - lo) / (hi - lo)) * h;
    };

    // Line
    ctx.strokeStyle = "white";
    ctx.lineWidth = 2;
    ctx.beginPath();
    series.forEach((p, i) => {
      const x = i * xStep;
      const y = yMap(p.v);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Dot
    const last = series[series.length - 1];
    ctx.fillStyle = "white";
    ctx.beginPath();
    ctx.arc((series.length - 1) * xStep, yMap(last.v), 3.5, 0, Math.PI * 2);
    ctx.fill();
  }

  async function pingHealth() {
    try {
      const r = await fetch("/health", { cache: "no-store" });
      apiEl.textContent = r.ok ? "OK" : "ERR";
    } catch (_) {
      apiEl.textContent = "ERR";
    }
  }

  async function fetchData() {
    try {
      const r = await fetch("/api/tds", { cache: "no-store" });
      if (!r.ok) throw new Error("bad");
      const j = await r.json();

      const ppm = (j.tds_ppm == null) ? null : Number(j.tds_ppm);
      const volt = (j.voltage == null) ? null : Number(j.voltage);
      const age = (j.age_ms == null) ? null : Number(j.age_ms);
      const up  = (j.uptime_ms == null) ? null : Number(j.uptime_ms);
      const temp = (j.temp_c == null) ? null : Number(j.temp_c);
      const waterHeightInches = (j.water_level_inches == null) ? null : Number(j.water_level_inches);
      const waterLevelPercent = (j.water_level_percent == null) ? null : Number(j.water_level_percent);
      const waterVolumeLiters = (j.water_volume_liters == null) ? null : Number(j.water_volume_liters);
      const waterLevelLabel = j.water_level_label || "Unknown";
      const oddSwitch = (j.ladder_odd_switch == null) ? 0 : Number(j.ladder_odd_switch);
      const evenSwitch = (j.ladder_even_switch == null) ? 0 : Number(j.ladder_even_switch);
      const oddVoltage = (j.ladder_odd_voltage == null) ? null : Number(j.ladder_odd_voltage);
      const evenVoltage = (j.ladder_even_voltage == null) ? null : Number(j.ladder_even_voltage);
      const waterLevelAge = (j.water_level_age_ms == null) ? null : Number(j.water_level_age_ms);

      tdsEl.textContent = ppm == null ? "--" : Math.round(ppm).toString();
      voltEl.textContent = volt == null ? "--" : volt.toFixed(3);
      ageEl.textContent = age == null ? "--" : age.toFixed(0);
      uptimeEl.textContent = up == null ? "--" : msToHuman(up);

      tempEl.textContent = temp == null ? "--" : temp.toFixed(0);
      tempBar.style.width = Math.max(0, Math.min(100, temp == null ? 0 : (temp / 50) * 100)) + "%";
      waterLevelPercentEl.textContent = waterLevelPercent == null ? "--" : waterLevelPercent.toFixed(0);
      waterLevelLabelEl.textContent = waterLevelLabel;
      waterLevelLitersEl.textContent = waterVolumeLiters == null ? "--" : waterVolumeLiters.toFixed(1);
      waterLevelBar.style.width = Math.max(0, Math.min(100, waterLevelPercent ?? 0)) + "%";
      waterHeightValueEl.textContent = waterHeightInches == null ? "--" : `${waterHeightInches.toFixed(1)} in`;
      oddLevelValueEl.textContent = formatLadder("GPIO33", oddSwitch, oddVoltage);
      evenLevelValueEl.textContent = formatLadder("GPIO32", evenSwitch, evenVoltage);
      levelAgeValueEl.textContent = msToHuman(waterLevelAge);

      const q = quality(ppm);
      setPill(q);

      if (ppm != null && !Number.isNaN(ppm)) {
        series.push({ t: Date.now(), v: ppm });
        while (series.length > maxPoints) series.shift();
        drawChart();
      }

      wifiText.textContent = "Online";
      wifiChip.style.borderColor = "rgba(50,255,170,.35)";
    } catch (e) {
      wifiText.textContent = "Offline";
      wifiChip.style.borderColor = "rgba(255,90,120,.35)";
      apiEl.textContent = "ERR";
    }
  }

  function tickClock() {
    const d = new Date();
    dateText.textContent = d.toLocaleString(undefined, {
      year: "numeric", month: "2-digit", day: "2-digit",
      hour: "2-digit", minute: "2-digit"
    });
  }

  btnRefresh?.addEventListener("click", async () => {
    await pingHealth();
    await fetchData();
  });

  tickClock();
  setInterval(tickClock, 1000);

  // Start
  pingHealth();
  fetchData();
  setInterval(fetchData, REFRESH_MS);
})();
