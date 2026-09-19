"""Self-contained HTML preview of a built robot: orbit view + one slider per movable joint."""
from __future__ import annotations

import base64
import json

import numpy as np

from .builder import Robot, link_mesh

MAX_TRIS_PER_LINK = 60000


def _b64(a, dtype):
    return base64.b64encode(np.ascontiguousarray(a, dtype=dtype).tobytes()).decode()


def preview_data(robot: Robot) -> dict:
    s = robot.config.unit_scale
    links = {}
    for name, link in robot.links.items():
        v, f = link_mesh(link, s, quality=0.5)
        if len(f) > MAX_TRIS_PER_LINK:          # keep the page light: subsample triangles
            f = f[np.linspace(0, len(f) - 1, MAX_TRIS_PER_LINK).astype(int)]
        links[name] = {"pos": _b64(v, "<f4"), "idx": _b64(f.ravel(), "<u4"), "color": link.color[:3],
                       "com": ((link.com - link.frame) * s).tolist(), "mass": link.mass}
    joints = []
    for j in robot.joints:
        js, parent = j.spec, robot.links[j.spec.parent]
        lo, hi = js.lower, js.upper
        if js.type == "revolute":
            lo, hi = (-np.pi if lo is None else lo), (np.pi if hi is None else hi)
        elif js.type == "prismatic":
            lo, hi = (-0.1 if lo is None else lo), (0.1 if hi is None else hi)
        elif js.type == "continuous":
            lo, hi = -np.pi, np.pi
        joints.append({"name": js.name, "type": js.type, "parent": js.parent, "child": js.child,
                       "xyz": ((j.origin_world - parent.frame) * s).tolist(), "axis": j.axis.tolist(),
                       "lower": lo, "upper": hi})
    lo = np.min([p.bbox_min for l in robot.links.values() for p in l.parts], 0) * s
    hi = np.max([p.bbox_max for l in robot.links.values() for p in l.parts], 0) * s
    return {"root": robot.root, "links": links, "joints": joints,
            "center": ((lo + hi) / 2).tolist(), "size": float(np.linalg.norm(hi - lo)) or 1.0}


HTML = r"""<!doctype html><html><head><meta charset="utf-8"><title>__TITLE__ preview</title>
<style>
 html,body{margin:0;height:100%;background:#eef1f4;font:14px/1.4 "IBM Plex Sans",system-ui,sans-serif;color:#1d2a36}
 #wrap{display:flex;height:100%}
 #view{flex:1;min-width:0;position:relative}
 #panel{width:260px;padding:14px 16px;overflow:auto;background:#fff;border-left:1px solid #d5dbe1}
 h2{font-size:15px;margin:0 0 10px}
 .j{margin:0 0 14px}
 .j label{display:flex;justify-content:space-between;font-size:13px}
 .j small{color:#5f6f7e}
 input[type=range]{width:100%}
 button{font:inherit;padding:5px 10px;border:1px solid #b9c3cc;background:#f7f9fa;border-radius:4px;cursor:pointer}
 button:focus-visible,input:focus-visible{outline:2px solid #1f6feb;outline-offset:2px}
 #hint{position:absolute;left:10px;bottom:8px;font-size:12px;color:#5f6f7e}
 .row{display:flex;gap:6px;flex-wrap:wrap;margin:0 0 12px}
 @media (max-width:640px){#wrap{flex-direction:column}#panel{width:auto;height:40%;border-left:0;border-top:1px solid #d5dbe1}}
</style></head><body><div id="wrap"><div id="view"><div id="hint">Drag to orbit, right-drag to pan, scroll to zoom</div></div>
<div id="panel"><h2>__TITLE__</h2><div class="row"><button id="zero">Zero pose</button><button id="axes">Hide axes</button></div><div id="sl"></div></div></div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
const D = __DATA__;
const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
view.appendChild(renderer.domElement);
// orbit state must exist before any slider calls draw()
const target = new THREE.Vector3(...D.center); let rad = D.size*1.6, th = -Math.PI/3, ph = 1.1;
function place(){ cam.position.set(target.x+rad*Math.sin(ph)*Math.cos(th), target.y+rad*Math.sin(ph)*Math.sin(th), target.z+rad*Math.cos(ph)); cam.lookAt(target); }
function draw(){ place(); renderer.render(scene, cam); }
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xeef1f4);
var cam = new THREE.PerspectiveCamera(40, 1, D.size/1000, D.size*100); cam.up.set(0,0,1);
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a96a3, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(1,-2,3); scene.add(sun);
const grid = new THREE.GridHelper(D.size*2, 20, 0x9aa6b1, 0xc8d0d7); grid.rotation.x = Math.PI/2; scene.add(grid);
const dec = s => Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer;
const groups = {}, axesHelpers = [];
for (const [name, L] of Object.entries(D.links)) {
  const g = new THREE.Group(); g.name = name; groups[name] = g;
  const pos = new Float32Array(dec(L.pos)), idx = new Uint32Array(dec(L.idx));
  if (idx.length) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(new THREE.BufferAttribute(idx, 1)); geo.computeVertexNormals();
    g.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({color:new THREE.Color(...L.color), metalness:0.1, roughness:0.6, side:THREE.DoubleSide})));
  }
}
scene.add(groups[D.root]);
const movers = [];
for (const j of D.joints) {
  const jg = new THREE.Group(); jg.position.set(...j.xyz);
  groups[j.parent].add(jg);
  const inner = new THREE.Group(); jg.add(inner); inner.add(groups[j.child]);
  if (j.type !== 'fixed') {
    const ax = new THREE.Vector3(...j.axis).normalize();
    const arrow = new THREE.ArrowHelper(ax, new THREE.Vector3(), D.size*0.28, j.type==='prismatic'?0x0a7d4f:0xc2185b, D.size*0.03, D.size*0.015);
    arrow.traverse(o => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 999; });
    jg.add(arrow); axesHelpers.push(arrow);
    movers.push({j, inner, ax});
  }
}
const sl = document.getElementById('sl');
if (!movers.length) sl.innerHTML = '<p><small>All joints are fixed. Set a joint to revolute, continuous or prismatic to move it here.</small></p>';
for (const m of movers) {
  const unit = m.j.type === 'prismatic' ? 'm' : 'rad';
  const d = document.createElement('div'); d.className = 'j';
  d.innerHTML = `<label><span>${m.j.name}</span><span class="v">0</span></label><input type="range" min="${m.j.lower}" max="${m.j.upper}" step="${(m.j.upper-m.j.lower)/400}" value="${Math.min(Math.max(0,m.j.lower),m.j.upper)}"><small>${m.j.type}, ${m.j.parent} to ${m.j.child}</small>`;
  sl.appendChild(d);
  const r = d.querySelector('input'), v = d.querySelector('.v');
  m.set = q => { r.value = q; v.textContent = (+q).toFixed(3)+' '+unit;
    if (m.j.type === 'prismatic') { m.inner.position.copy(m.ax).multiplyScalar(+q); }
    else m.inner.quaternion.setFromAxisAngle(m.ax, +q); draw(); };
  r.addEventListener('input', () => m.set(r.value));
  m.set(r.value);
}
document.getElementById('zero').onclick = () => movers.forEach(m => m.set(Math.min(Math.max(0,m.j.lower),m.j.upper)));
document.getElementById('axes').onclick = e => { const on = !axesHelpers[0]?.visible; axesHelpers.forEach(a=>a.visible=on); e.target.textContent = on?'Hide axes':'Show axes'; draw(); };
// minimal orbit controls (z-up)
let drag = null;
renderer.domElement.addEventListener('pointerdown', e => { drag = {x:e.clientX, y:e.clientY, b:e.button}; renderer.domElement.setPointerCapture(e.pointerId); });
renderer.domElement.addEventListener('pointerup', () => drag = null);
renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());
renderer.domElement.addEventListener('pointermove', e => { if (!drag) return;
  const dx = e.clientX-drag.x, dy = e.clientY-drag.y; drag.x = e.clientX; drag.y = e.clientY;
  if (drag.b === 2 || e.shiftKey) { const k = rad/400, right = new THREE.Vector3(), up = new THREE.Vector3();
    cam.matrixWorld.extractBasis(right, up, new THREE.Vector3()); target.addScaledVector(right, -dx*k).addScaledVector(up, dy*k); }
  else { th -= dx*0.008; ph = Math.min(Math.max(ph - dy*0.008, 0.05), Math.PI-0.05); }
  draw(); });
renderer.domElement.addEventListener('wheel', e => { e.preventDefault(); rad *= Math.exp(e.deltaY*0.001); draw(); }, {passive:false});
function resize(){ const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h); cam.aspect = w/h; cam.updateProjectionMatrix(); draw(); }
window.addEventListener('resize', resize); resize();
</script></body></html>"""


def preview_html(robot: Robot) -> str:
    data = json.dumps(preview_data(robot), separators=(",", ":"))
    return HTML.replace("__TITLE__", robot.config.robot_name).replace("__DATA__", data)
