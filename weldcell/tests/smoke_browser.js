#!/usr/bin/env node
/* Browser smoke test (Playwright/Chromium): loads index.html over file://,
 * drives every stage through window.CobotAPI / the UI and fails on any console error.
 *   node tests/smoke_browser.js [outDir]          (screenshots -> outDir)   */
'use strict';
const path = require('path'), fs = require('fs');
const { chromium } = require('playwright');
const out = process.argv[2] || path.join(__dirname, '..', 'tests', 'screenshots');
fs.mkdirSync(out, { recursive: true });
const url = 'file://' + path.join(__dirname, '..', 'index.html');
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

(async () => {
  const browser = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const page = await browser.newPage({ viewport: { width: 1600, height: 960 }, deviceScaleFactor: 1 });
  const errors = [];
  page.on('console', m => { if (m.type() === 'error' && !/fonts\.(googleapis|gstatic)/.test(m.text())) errors.push(m.text()); });
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));
  page.on('requestfailed', r => { if (!/fonts\.(googleapis|gstatic)/.test(r.url())) errors.push('requestfailed: ' + r.url()); });
  await page.goto(url + '#stage=1'); await sleep(1500);
  const shot = async (n) => page.screenshot({ path: path.join(out, n + '.png') });
  const check = async (label, fn) => { const r = await page.evaluate(fn); console.log(`  ${label}:`, JSON.stringify(r)); return r; };

  await shot('1_plan');
  // ② placement + reachability
  await page.click('.step[data-s="2"]'); await sleep(600);
  await check('reach', () => document.getElementById('reachOut').textContent);
  await shot('2_place');
  // ③ weld with AUTO 3-axis
  await page.click('.step[data-s="3"]'); await sleep(200);
  await page.evaluate(() => { CobotAPI.approach(); });
  await sleep(1500); await page.evaluate(() => CobotAPI.startWeld()); await sleep(4000);
  const st = await check('weld state', () => ({ phase, s: +s.toFixed(3), pool: CobotAPI.getPoolObs().status, A: +(CobotAPI.getPoolObs().area * 1e6).toFixed(1) }));
  if (st.pool !== 'ok') errors.push('pool not ok during weld');
  await shot('3_weld');
  for (const t of ['pool', 'thermal', 'quality']) { await page.click(`.tab[data-p="${t}"]`); await sleep(500); await shot('3_tab_' + t); }
  // ④ learning (small budget)
  await page.click('.step[data-s="4"]'); await sleep(200);
  await page.evaluate(() => { CobotAPI.setParam('iters', 4); CobotAPI.setParam('pop', 12); CobotAPI.setParam('epSteps', 160); });
  await page.click('#trainBtn');
  await page.waitForFunction(() => !WeldCellApp.RL.busy && WeldCellApp.RL.policy, null, { timeout: 120000 });
  await check('eval', () => (WeldCellApp.RL.evalRows || []).map(r => [r.name, +r.meanReward.toFixed(3)]));
  await page.click('#deployBtn'); await sleep(200);
  await page.evaluate(() => { CobotAPI.reset(); }); await sleep(1500);
  await page.evaluate(() => { CobotAPI.approach(); }); await sleep(1500); await page.evaluate(() => CobotAPI.startWeld()); await sleep(3000);
  await check('policy weld', () => ({ ctl: CH.ctlMode, phase, pool: CobotAPI.getPoolObs().status, v: +(CobotAPI.getWeldInput().travelSpeed * 1000).toFixed(2) }));
  await page.evaluate(() => { document.getElementById('coreRunBtn').click(); document.getElementById('coreEnvBtn').click(); }); await sleep(300);
  await check('core', () => document.getElementById('coreOut').textContent);
  await shot('4_learn');
  // ⑤ Sim⇄Real with the virtual real machine
  await page.click('.step[data-s="5"]'); await sleep(200);
  await page.click('#connBtn'); await sleep(2500);
  await check('conn', () => document.getElementById('connOut').textContent.split('\n')[0]);
  await page.evaluate(() => CobotAPI.setChoice('opMode', 'control')); await page.click('#armBtn');
  await page.evaluate(() => CobotAPI.setParam('exciteT', 10)); await page.click('#exciteBtn'); await sleep(11500);
  await page.click('#calFromLog'); await sleep(800);
  await check('calib', () => document.getElementById('calOut').textContent);
  await page.click('#calApply'); await sleep(2500);
  await check('residual', () => document.getElementById('twR').textContent);
  await page.click('#calSynth'); await sleep(800);
  await shot('5_sim2real');
  await page.click('#estopBtn'); await sleep(300);
  await check('estop', () => ({ safe: document.getElementById('twSafe').textContent, backendEstop: WeldCellApp.S2R.backend && WeldCellApp.S2R.backend.estopped }));
  await page.click('.tab[data-p="log"]'); await sleep(300); await shot('6_log');
  await browser.close();
  if (errors.length) { console.log('\nCONSOLE ERRORS:\n' + errors.join('\n')); process.exit(1); }
  console.log('\nsmoke OK — screenshots in ' + out);
})().catch(e => { console.error(e); process.exit(1); });
