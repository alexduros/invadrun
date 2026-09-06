/* invadrun SVG map: Paris context + route coloured by stage. No tiles, no library. */
(function (global) {
  const STAGE_VARS = 8;
  const swatch = (n) => `var(--s${((n - 1) % STAGE_VARS) + 1})`;

  function bounds(plan) {
    let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    const eat = (lon, lat) => { if (lat < minLat) minLat = lat; if (lat > maxLat) maxLat = lat; if (lon < minLon) minLon = lon; if (lon > maxLon) maxLon = lon; };
    const walk = (c) => (typeof c[0] === "number" ? eat(c[0], c[1]) : c.forEach(walk));
    plan.context.features.filter((f) => f.properties.kind === "city").forEach((f) => walk(f.geometry.coordinates));
    plan.polyline.forEach(([lat, lon]) => eat(lon, lat));
    const padLat = (maxLat - minLat) * 0.03, padLon = (maxLon - minLon) * 0.03;
    return { minLat: minLat - padLat, maxLat: maxLat + padLat, minLon: minLon - padLon, maxLon: maxLon + padLon };
  }

  function projector(b, W) {
    const lat0 = (b.minLat + b.maxLat) / 2, kx = Math.cos((lat0 * Math.PI) / 180);
    const s = W / ((b.maxLon - b.minLon) * kx);
    const H = Math.round((b.maxLat - b.minLat) * s);
    return { W, H, X: (lon) => (lon - b.minLon) * kx * s, Y: (lat) => (b.maxLat - lat) * s };
  }

  function pathFor(geom, P) {
    const ring = (c) => c.map(([lon, lat], i) => (i ? "L" : "M") + P.X(lon).toFixed(1) + " " + P.Y(lat).toFixed(1)).join("");
    switch (geom.type) {
      case "Polygon": return geom.coordinates.map((r) => ring(r) + "Z").join("");
      case "MultiPolygon": return geom.coordinates.flat().map((r) => ring(r) + "Z").join("");
      case "LineString": return ring(geom.coordinates);
      case "MultiLineString": return geom.coordinates.map(ring).join("");
      default: return "";
    }
  }

  function centroid(geom) {
    const rings = geom.type === "Polygon" ? [geom.coordinates[0]] : geom.coordinates.map((p) => p[0]);
    let big = rings[0];
    rings.forEach((r) => { if (r.length > big.length) big = r; });
    let a = 0, cx = 0, cy = 0;
    for (let i = 0; i < big.length - 1; i++) {
      const [x1, y1] = big[i], [x2, y2] = big[i + 1], f = x1 * y2 - x2 * y1;
      a += f; cx += (x1 + x2) * f; cy += (y1 + y2) * f;
    }
    a *= 0.5; return a ? [cx / (6 * a), cy / (6 * a)] : big[0];
  }

  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  /** Render into `el`. opts: {width, labels, dots, legend, pins, title} */
  function render(el, plan, opts = {}) {
    const W = opts.width || 1000;
    const b = bounds(plan), P = projector(b, W);
    const ctx = plan.context.features;
    const layers = [];

    const water = ctx.filter((f) => f.properties.kind === "water");
    if (water.length) layers.push(`<g class="m-water">${water.map((f) => `<path d="${pathFor(f.geometry, P)}"/>`).join("")}</g>`);
    const river = ctx.filter((f) => f.properties.kind === "river");
    if (river.length && !water.length) layers.push(`<g class="m-river">${river.map((f) => `<path d="${pathFor(f.geometry, P)}"/>`).join("")}</g>`);

    const arr = ctx.filter((f) => f.properties.kind === "arrondissement");
    layers.push(`<g class="m-arr">${arr.map((f) => `<path d="${pathFor(f.geometry, P)}"/>`).join("")}</g>`);
    ctx.filter((f) => f.properties.kind === "city").forEach((f) => layers.push(`<path class="m-city" d="${pathFor(f.geometry, P)}"/>`));
    if (opts.labels !== false) {
      layers.push(`<g class="m-arr-labels">${arr.map((f) => { const [lon, lat] = centroid(f.geometry); return `<text x="${P.X(lon).toFixed(1)}" y="${P.Y(lat).toFixed(1)}">${f.properties.number}</text>`; }).join("")}</g>`);
    }

    // route, one path per stage (stages share boundary points)
    const poly = plan.polyline;
    layers.push(`<g class="m-route">${plan.stages.map((s) => {
      const pts = poly.slice(s.poly_start, Math.min(s.poly_end + 1, poly.length));
      const d = pts.map(([lat, lon], i) => (i ? "L" : "M") + P.X(lon).toFixed(1) + " " + P.Y(lat).toFixed(1)).join("");
      return `<path class="m-st m-st-${s.n}" data-stage="${s.n}" d="${d}" style="stroke:${swatch(s.n)}"/>`;
    }).join("")}</g>`);

    if (opts.dots !== false) {
      layers.push(`<g class="m-dots">${plan.route.map((r) => `<circle class="m-dot m-st-${r.stage}" data-stage="${r.stage}" cx="${P.X(r.lon).toFixed(1)}" cy="${P.Y(r.lat).toFixed(1)}" r="1.7"><title>#${r.i} ${esc(r.label)} — ${esc(r.address)} (${r.cum_km} km)</title></circle>`).join("")}</g>`);
    }

    if (opts.pins !== false) {
      const pins = plan.stages.map((s) => ({ ...s.start, n: s.n }));
      const fin = plan.meta.end;
      layers.push(`<g class="m-pins">${pins.map((p) => `<g class="m-pin m-st-${p.n}" data-stage="${p.n}" transform="translate(${P.X(p.lon).toFixed(1)} ${P.Y(p.lat).toFixed(1)})"><circle r="9" style="fill:${swatch(p.n)}"/><text y="3.6">${p.n}</text><title>Stage ${p.n} starts here: ${esc(p.label)}, ${esc(p.address)}</title></g>`).join("")}<g class="m-pin m-fin" transform="translate(${P.X(fin.lon).toFixed(1)} ${P.Y(fin.lat).toFixed(1)})"><rect x="-9" y="-9" width="18" height="18" rx="2"/><text y="3.6">F</text><title>Finish: ${esc(fin.label)}, ${esc(fin.address)} — ${fin.cum_km} km</title></g></g>`);
    }

    el.innerHTML = `<svg class="m-svg" viewBox="0 0 ${P.W} ${P.H}" role="img" aria-label="${esc(opts.title || "Route through Paris, coloured by stage")}" xmlns="http://www.w3.org/2000/svg">${layers.join("")}</svg>`;
    return { svg: el.firstElementChild, P, bounds: b };
  }

  /** Dim every stage except `n` (null = show all). */
  function focus(el, n) {
    el.querySelectorAll("[data-stage]").forEach((node) => node.classList.toggle("m-dim", n != null && +node.dataset.stage !== +n));
  }

  global.InvMap = { render, focus, swatch };
})(window);
