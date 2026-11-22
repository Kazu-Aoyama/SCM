/* SCMIO — SCMリカレント教育ツール共通の入出力モジュール
 * ・Excel（.xlsx）の読み込み・書き出し（外部ライブラリなし。ブラウザ標準の DecompressionStream を使う）
 * ・CSV の読み込み（UTF-8／Shift_JIS 自動判定）・書き出し
 * ・ツール間の受け渡し（同じブラウザ内：localStorage／開いたウィンドウ：postMessage）
 * データは端末の外へ一切送らない。
 * このファイルは src/scm-io.js が原本。build.py が各ツールの SCMIO の目印のあいだに埋め込む（ツール側で直接直さない）。
 */
(function (root) {
  'use strict';
  var VERSION = '1.0.0';

  /* ---------- ZIP（読み） ---------- */
  function u16(b, o) { return b[o] | (b[o + 1] << 8); }
  function u32(b, o) { return (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0; }

  async function inflateRaw(bytes) {
    if (typeof DecompressionStream === 'undefined') throw new Error('このブラウザは xlsx の展開に対応していません（Chrome・Edge・Firefox・Safari の最新版を使ってください）');
    var ds = new DecompressionStream('deflate-raw');
    var stream = new Blob([bytes]).stream().pipeThrough(ds);
    return new Uint8Array(await new Response(stream).arrayBuffer());
  }

  async function unzip(buf) {
    var b = new Uint8Array(buf), eocd = -1;
    for (var i = b.length - 22; i >= Math.max(0, b.length - 65557); i--) {
      if (u32(b, i) === 0x06054b50) { eocd = i; break; }
    }
    if (eocd < 0) throw new Error('xlsx（ZIP）として読めません。Excel で「名前を付けて保存 → Excel ブック（.xlsx）」で保存し直してください');
    var n = u16(b, eocd + 10), p = u32(b, eocd + 16), files = {}, dec = new TextDecoder('utf-8');
    for (var k = 0; k < n; k++) {
      if (u32(b, p) !== 0x02014b50) break;
      var method = u16(b, p + 10), csize = u32(b, p + 20), nl = u16(b, p + 28), xl = u16(b, p + 30), cl = u16(b, p + 32), lho = u32(b, p + 42);
      var name = dec.decode(b.subarray(p + 46, p + 46 + nl));
      files[name] = { method: method, csize: csize, lho: lho };
      p += 46 + nl + xl + cl;
    }
    return {
      names: Object.keys(files),
      has: function (nm) { return !!files[nm]; },
      text: async function (nm) {
        var f = files[nm]; if (!f) return null;
        var s = f.lho + 30 + u16(b, f.lho + 26) + u16(b, f.lho + 28);
        var raw = b.subarray(s, s + f.csize);
        var out = f.method === 0 ? raw : f.method === 8 ? await inflateRaw(raw) : null;
        if (!out) throw new Error('未対応の圧縮方式です: ' + nm);
        return dec.decode(out);
      }
    };
  }

  /* ---------- xlsx（読み） ---------- */
  function xml(s) { return new DOMParser().parseFromString(s, 'application/xml'); }
  function kids(el, tag) { var r = [], c = el ? el.childNodes : []; for (var i = 0; i < c.length; i++) if (c[i].localName === tag) r.push(c[i]); return r; }
  function all(doc, tag) { return Array.prototype.slice.call(doc.getElementsByTagNameNS('*', tag)); }
  function textOf(el) {
    // <si> / <is> は <t> と <r><t> の連結。<rPh>（ふりがな）は除く
    var s = '';
    (function walk(n) {
      for (var i = 0; i < n.childNodes.length; i++) {
        var c = n.childNodes[i];
        if (c.localName === 'rPh') continue;
        if (c.localName === 't') s += c.textContent; else if (c.nodeType === 1) walk(c);
      }
    })(el);
    return s;
  }
  function colIndex(ref) { var m = /^([A-Z]+)(\d+)$/.exec(ref); if (!m) return null; var c = 0; for (var i = 0; i < m[1].length; i++) c = c * 26 + (m[1].charCodeAt(i) - 64); return { c: c - 1, r: +m[2] - 1 }; }
  var BUILTIN_DATE = { 14: 1, 15: 1, 16: 1, 17: 1, 18: 2, 19: 2, 20: 2, 21: 2, 22: 1, 27: 1, 28: 1, 29: 1, 30: 1, 31: 1, 32: 2, 33: 2, 34: 2, 35: 2, 36: 1, 45: 2, 46: 2, 47: 2, 50: 1, 51: 1, 52: 1, 53: 1, 54: 1, 55: 1, 56: 1, 57: 1, 58: 1 };
  function isDateFmt(code) {
    // 書式コードから文字列・色指定・エスケープ・General・指数表記を除き，日付時刻の記号が残れば日付とみなす
    var t = String(code).replace(/"[^"]*"/g, '').replace(/\\./g, '').replace(/_.|\*./g, '').replace(/General/gi, '')
      .replace(/\[(h|hh|m|mm|s|ss)\]/gi, 'h').replace(/\[[^\]]*\]/g, '').replace(/[0#?.,]+E[+-]\d+/gi, '').replace(/AM\/PM|A\/P/gi, 'h');
    if (!/[ymdhsge]/i.test(t)) return 0;
    // y・d・年号があれば日付。m だけで h・s がなければ月（日付），h・s があれば時刻
    return (/[ydge]/i.test(t) || (/m/i.test(t) && !/[hs]/i.test(t))) ? 1 : 2;
  }
  function serialToISO(v, kind, date1904) {
    var ms = Math.round((v + (date1904 ? 1462 : 0) - 25569) * 86400000);
    var d = new Date(ms), iso = d.toISOString();
    if (kind === 2 && v < 1) return iso.substr(11, 8);
    if (d.getUTCHours() === 0 && d.getUTCMinutes() === 0 && d.getUTCSeconds() === 0) return iso.substr(0, 10);
    return iso.substr(0, 10) + ' ' + iso.substr(11, 8);
  }

  /* シート XML を DOM を使わずに走査する（大きな表でも速い） */
  function unesc(t) { return t.indexOf('&') < 0 ? t : t.replace(/&(lt|gt|amp|quot|apos|#x?[0-9a-fA-F]+);/g, function (m, e) { return e === 'lt' ? '<' : e === 'gt' ? '>' : e === 'amp' ? '&' : e === 'quot' ? '"' : e === 'apos' ? "'" : String.fromCodePoint(e[1] === 'x' ? parseInt(e.slice(2), 16) : +e.slice(1)); }); }
  function attr(tag, name) { var m = new RegExp('\\s' + name + '="([^"]*)"').exec(tag); return m ? m[1] : null; }
  function parseSheetXml(txt, sst, styleKind, date1904) {
    var rows = [], i0 = txt.indexOf('<sheetData'), end = txt.indexOf('</sheetData>');
    if (i0 < 0) return rows; if (end < 0) end = txt.length;
    var re = /<(?:\w+:)?(row|c)\b([^>]*?)(\/?)>/g, m, row = null, rIdx = -1, cIdx = -1;
    re.lastIndex = i0;
    while ((m = re.exec(txt)) && m.index < end) {
      if (m[1] === 'row') { var r = attr(m[2], 'r'); rIdx = r ? +r - 1 : rIdx + 1; row = rows[rIdx] = []; cIdx = -1; continue; }
      var ref = attr(m[2], 'r'), pos = ref ? colIndex(ref) : null; cIdx = pos ? pos.c : cIdx + 1;
      if (!row) { rIdx++; row = rows[rIdx] = []; }
      if (m[3] === '/') { row[cIdx] = null; continue; }
      var close = txt.indexOf('</c>', re.lastIndex); if (close < 0) close = txt.indexOf('/c>', re.lastIndex);
      var inner = txt.slice(re.lastIndex, close); re.lastIndex = close + 4;
      var t = attr(m[2], 't') || 'n', vm = /<(?:\w+:)?v>([\s\S]*?)<\/(?:\w+:)?v>/.exec(inner), v = vm ? unesc(vm[1]) : null, val = null;
      if (t === 's') val = v == null ? null : sst[+v];
      else if (t === 'inlineStr') { val = ''; inner.replace(/<(?:\w+:)?rPh\b[\s\S]*?<\/(?:\w+:)?rPh>/g, '').replace(/<(?:\w+:)?t\b[^>]*>([\s\S]*?)<\/(?:\w+:)?t>/g, function (_, x) { val += unesc(x); }); }
      else if (t === 'str') val = v;
      else if (t === 'b') val = v === '1';
      else if (t === 'e') val = null;
      else if (t === 'd') val = v;
      else if (v != null && v !== '') { val = +v; var kind = styleKind[+(attr(m[2], 's') || 0)]; if (kind) val = serialToISO(val, kind, date1904); }
      row[cIdx] = val;
    }
    return rows;
  }

  async function readXlsx(buf) {
    var z = await unzip(buf);
    if (!z.has('xl/workbook.xml')) throw new Error('Excel ブック（.xlsx）ではありません。古い .xls は Excel で .xlsx に保存し直してください');
    var wb = xml(await z.text('xl/workbook.xml'));
    var date1904 = all(wb, 'workbookPr').some(function (e) { var v = e.getAttribute('date1904'); return v === '1' || v === 'true'; });
    var rels = {}, relsTxt = await z.text('xl/_rels/workbook.xml.rels');
    if (relsTxt) all(xml(relsTxt), 'Relationship').forEach(function (r) { rels[r.getAttribute('Id')] = r.getAttribute('Target'); });
    var sst = [], sstTxt = await z.text('xl/sharedStrings.xml');
    if (sstTxt) sst = all(xml(sstTxt), 'si').map(textOf);
    var styleKind = [], stTxt = await z.text('xl/styles.xml');
    if (stTxt) {
      var st = xml(stTxt), custom = {};
      all(st, 'numFmt').forEach(function (f) { custom[f.getAttribute('numFmtId')] = isDateFmt(f.getAttribute('formatCode') || ''); });
      var xfs = all(st, 'cellXfs')[0];
      kids(xfs, 'xf').forEach(function (x) { var id = +x.getAttribute('numFmtId'); styleKind.push(id in custom ? custom[id] : (BUILTIN_DATE[id] || 0)); });
    }
    var sheets = [];
    for (var s of all(wb, 'sheet')) {
      var rid = s.getAttribute('r:id') || s.getAttributeNS('http://schemas.openxmlformats.org/officeDocument/2006/relationships', 'id');
      var target = rels[rid] || ''; target = target.replace(/^\/?xl\//, '').replace(/^\//, '');
      var path = 'xl/' + target, txt = await z.text(path);
      if (!txt) continue;
      var rows = parseSheetXml(txt, sst, styleKind, date1904);
      for (var i = 0; i < rows.length; i++) { if (!rows[i]) rows[i] = []; for (var j = 0; j < rows[i].length; j++) if (rows[i][j] === undefined) rows[i][j] = null; }
      sheets.push({ name: s.getAttribute('name'), rows: rows });
    }
    return sheets;
  }

  /* ---------- CSV ---------- */
  function decodeText(buf) {
    var b = new Uint8Array(buf);
    if (b[0] === 0xEF && b[1] === 0xBB && b[2] === 0xBF) return new TextDecoder('utf-8').decode(b.subarray(3));
    if (b[0] === 0xFF && b[1] === 0xFE) return new TextDecoder('utf-16le').decode(b.subarray(2));
    try { return new TextDecoder('utf-8', { fatal: true }).decode(b); }
    catch (e) { return new TextDecoder('shift_jis').decode(b); }
  }
  function parseCsv(text, delim) {
    if (!delim) { var first = text.split(/\r?\n/, 1)[0]; delim = (first.split('\t').length > first.split(',').length) ? '\t' : ','; }
    var rows = [], row = [], f = '', q = false, i = 0, n = text.length;
    for (; i < n; i++) {
      var ch = text[i];
      if (q) { if (ch === '"') { if (text[i + 1] === '"') { f += '"'; i++; } else q = false; } else f += ch; }
      else if (ch === '"') q = true;
      else if (ch === delim) { row.push(f); f = ''; }
      else if (ch === '\n' || ch === '\r') { if (ch === '\r' && text[i + 1] === '\n') i++; row.push(f); rows.push(row); row = []; f = ''; }
      else f += ch;
    }
    if (f !== '' || row.length) { row.push(f); rows.push(row); }
    return rows.map(function (r) { return r.map(function (v) { if (v === '') return null; var t = v.trim(); return /^-?\d+(\.\d+)?([eE][-+]?\d+)?$/.test(t) ? +t : v; }); });
  }
  function toCsv(rows) {
    return '﻿' + rows.map(function (r) { return r.map(function (v) { if (v == null) return ''; var s = String(v); return /[",\r\n]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s; }).join(','); }).join('\r\n');
  }

  async function readFile(file) {
    var buf = await file.arrayBuffer(), name = (file.name || '').toLowerCase();
    if (/\.(xlsx|xlsm)$/.test(name)) return readXlsx(buf);
    if (/\.xls$/.test(name)) throw new Error('古い Excel 形式（.xls）は読めません。Excel で .xlsx として保存し直してください');
    var b = new Uint8Array(buf);
    if (b[0] === 0x50 && b[1] === 0x4B) return readXlsx(buf);
    return [{ name: file.name.replace(/\.[^.]+$/, ''), rows: parseCsv(decodeText(buf)) }];
  }

  /* ---------- 表 → オブジェクト ---------- */
  function findHeaderRow(rows) {
    // 先頭10行のうち，文字の入ったセルが最も多い行を見出しとみなす
    var best = 0, bestN = -1;
    for (var i = 0; i < Math.min(10, rows.length); i++) {
      var n = (rows[i] || []).filter(function (v) { return typeof v === 'string' && v.trim(); }).length;
      if (n > bestN) { bestN = n; best = i; }
    }
    return best;
  }
  function toObjects(rows, headerRow) {
    if (headerRow == null) headerRow = findHeaderRow(rows);
    var head = (rows[headerRow] || []).map(function (h, i) { return h == null || h === '' ? '列' + (i + 1) : String(h).trim(); });
    var out = [];
    for (var r = headerRow + 1; r < rows.length; r++) {
      var row = rows[r]; if (!row || row.every(function (v) { return v == null || v === ''; })) continue;
      var o = {}; head.forEach(function (h, i) { o[h] = row[i] == null ? null : row[i]; }); out.push(o);
    }
    return { columns: head, rows: out };
  }

  /* ---------- 列の対応づけ（見出しの別名から自動推定） ---------- */
  function norm(s) { return String(s || '').toLowerCase().replace(/[\s_\-・()（）［］\[\]／/.]/g, ''); }
  function guessMapping(columns, fields) {
    // fields: [{key, label, aliases:[...], required}]
    var map = {}, used = {};
    fields.forEach(function (f) {
      var cands = [f.key, f.label].concat(f.aliases || []).map(norm), hit = null;
      columns.forEach(function (c) { if (!hit && !used[c] && cands.indexOf(norm(c)) >= 0) hit = c; });
      if (!hit) columns.forEach(function (c) { if (!hit && !used[c] && cands.some(function (a) { return a && (norm(c).indexOf(a) >= 0 || a.indexOf(norm(c)) >= 0) && norm(c).length > 1; })) hit = c; });
      if (hit) { map[f.key] = hit; used[hit] = 1; }
    });
    return map;
  }

  /* ---------- 値の変換 ---------- */
  function toDate(v) {
    if (v == null || v === '') return null;
    if (typeof v === 'number') { if (v > 20000 && v < 80000) return serialToISO(v, 1, false).substr(0, 10); if (v > 19000101 && v < 21001231) { var s = String(v); return s.substr(0, 4) + '-' + s.substr(4, 2) + '-' + s.substr(6, 2); } return null; }
    var t = String(v).trim().replace(/[年月\/.]/g, '-').replace(/日/, '').replace(/[０-９]/g, function (c) { return String.fromCharCode(c.charCodeAt(0) - 0xFEE0); });
    var m = /^(\d{4})-(\d{1,2})(?:-(\d{1,2}))?/.exec(t);
    if (!m) return null;
    return m[1] + '-' + ('0' + m[2]).slice(-2) + '-' + ('0' + (m[3] || 1)).slice(-2);
  }
  function toNum(v) {
    if (v == null || v === '') return null; if (typeof v === 'number') return isFinite(v) ? v : null; if (typeof v === 'boolean') return v ? 1 : 0;
    var t = String(v).replace(/[０-９．－]/g, function (c) { return String.fromCharCode(c.charCodeAt(0) - 0xFEE0); }).replace(/[,，円個台%％\s]/g, '').replace(/^▲|^△/, '-');
    var n = parseFloat(t); return isFinite(n) ? n : null;
  }

  /* ---------- xlsx（書き） 無圧縮 ZIP ---------- */
  var CRC = (function () { var t = new Uint32Array(256); for (var n = 0; n < 256; n++) { var c = n; for (var k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
  function crc32(b) { var c = 0xFFFFFFFF; for (var i = 0; i < b.length; i++) c = CRC[(c ^ b[i]) & 0xFF] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; }
  function zipStore(entries) {
    var enc = new TextEncoder(), parts = [], cen = [], off = 0;
    entries.forEach(function (e) {
      var name = enc.encode(e.name), data = typeof e.data === 'string' ? enc.encode(e.data) : e.data, crc = crc32(data);
      var h = new DataView(new ArrayBuffer(30));
      h.setUint32(0, 0x04034b50, true); h.setUint16(4, 20, true); h.setUint16(6, 0x0800, true); h.setUint16(8, 0, true);
      h.setUint32(14, crc, true); h.setUint32(18, data.length, true); h.setUint32(22, data.length, true); h.setUint16(26, name.length, true);
      parts.push(new Uint8Array(h.buffer), name, data);
      var c = new DataView(new ArrayBuffer(46));
      c.setUint32(0, 0x02014b50, true); c.setUint16(4, 20, true); c.setUint16(6, 20, true); c.setUint16(8, 0x0800, true);
      c.setUint32(16, crc, true); c.setUint32(20, data.length, true); c.setUint32(24, data.length, true); c.setUint16(28, name.length, true); c.setUint32(42, off, true);
      cen.push(new Uint8Array(c.buffer), name);
      off += 30 + name.length + data.length;
    });
    var csize = cen.reduce(function (s, a) { return s + a.length; }, 0), e = new DataView(new ArrayBuffer(22));
    e.setUint32(0, 0x06054b50, true); e.setUint16(8, entries.length, true); e.setUint16(10, entries.length, true); e.setUint32(12, csize, true); e.setUint32(16, off, true);
    return new Blob(parts.concat(cen, [new Uint8Array(e.buffer)]), { type: 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet' });
  }
  function esc(s) { return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;').replace(/[\x00-\x08\x0B\x0C\x0E-\x1F]/g, ''); }
  function colName(i) { var s = ''; i++; while (i) { var m = (i - 1) % 26; s = String.fromCharCode(65 + m) + s; i = (i - m - 1) / 26; } return s; }
  function writeXlsx(sheets) {
    // sheets: [{name, rows:[[...]], widths?:[...]}]  1行目は見出し（太字・固定）。日付文字列 YYYY-MM-DD は日付セルにする
    var x = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n';
    var files = [];
    files.push({ name: '[Content_Types].xml', data: x + '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/><Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>' + sheets.map(function (s, i) { return '<Override PartName="/xl/worksheets/sheet' + (i + 1) + '.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'; }).join('') + '</Types>' });
    files.push({ name: '_rels/.rels', data: x + '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>' });
    files.push({ name: 'xl/workbook.xml', data: x + '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>' + sheets.map(function (s, i) { return '<sheet name="' + esc(String(s.name).replace(/[\\\/?*\[\]:]/g, '_').substr(0, 31)) + '" sheetId="' + (i + 1) + '" r:id="rId' + (i + 1) + '"/>'; }).join('') + '</sheets></workbook>' });
    files.push({ name: 'xl/_rels/workbook.xml.rels', data: x + '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' + sheets.map(function (s, i) { return '<Relationship Id="rId' + (i + 1) + '" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet' + (i + 1) + '.xml"/>'; }).join('') + '<Relationship Id="rId' + (sheets.length + 1) + '" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>' });
    files.push({ name: 'xl/styles.xml', data: x + '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><numFmts count="1"><numFmt numFmtId="164" formatCode="yyyy-mm-dd"/></numFmts><fonts count="3"><font><sz val="11"/><name val="Yu Gothic"/></font><font><b/><sz val="11"/><name val="Yu Gothic"/></font><font><i/><sz val="10"/><color rgb="FF666666"/><name val="Yu Gothic"/></font></fonts><fills count="3"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill><fill><patternFill patternType="solid"><fgColor rgb="FFDCEBF5"/></patternFill></fill></fills><borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders><cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs><cellXfs count="4"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/><xf numFmtId="0" fontId="1" fillId="2" borderId="0" xfId="0" applyFont="1" applyFill="1"/><xf numFmtId="164" fontId="0" fillId="0" borderId="0" xfId="0" applyNumberFormat="1"/><xf numFmtId="0" fontId="2" fillId="0" borderId="0" xfId="0" applyFont="1"/></cellXfs><cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles></styleSheet>' });
    sheets.forEach(function (s, si) {
      var maxC = s.rows.reduce(function (m, r) { return Math.max(m, r.length); }, 0);
      var widths = s.widths || [], cols = '';
      for (var c = 0; c < maxC; c++) {
        var w = widths[c] || Math.min(50, Math.max(8, s.rows.slice(0, 200).reduce(function (m, r) { var v = r[c]; if (v == null) return m; var t = String(v); var len = 0; for (var k = 0; k < t.length; k++) len += t.charCodeAt(k) > 255 ? 2 : 1; return Math.max(m, len); }, 0) + 2));
        cols += '<col min="' + (c + 1) + '" max="' + (c + 1) + '" width="' + w + '" customWidth="1"/>';
      }
      var body = s.rows.map(function (r, ri) {
        return '<row r="' + (ri + 1) + '">' + r.map(function (v, ci) {
          if (v == null || v === '') return '';
          var ref = colName(ci) + (ri + 1), st = ri === 0 && !s.noHeader ? ' s="1"' : (s.noteRows && s.noteRows.indexOf(ri) >= 0 ? ' s="3"' : '');
          if (typeof v === 'number' && isFinite(v)) return '<c r="' + ref + '"' + st + '><v>' + v + '</v></c>';
          if (typeof v === 'boolean') return '<c r="' + ref + '" t="b"' + st + '><v>' + (v ? 1 : 0) + '</v></c>';
          if (ri > 0 && /^\d{4}-\d{2}-\d{2}$/.test(v)) { var d = (Date.UTC(+v.substr(0, 4), +v.substr(5, 2) - 1, +v.substr(8, 2)) / 86400000) + 25569; return '<c r="' + ref + '" s="2"><v>' + d + '</v></c>'; }
          return '<c r="' + ref + '" t="inlineStr"' + st + '><is><t xml:space="preserve">' + esc(v) + '</t></is></c>';
        }).join('') + '</row>';
      }).join('');
      var pane = s.noHeader ? '' : '<sheetViews><sheetView workbookViewId="0"><pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/></sheetView></sheetViews>';
      files.push({ name: 'xl/worksheets/sheet' + (si + 1) + '.xml', data: x + '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">' + pane + (cols ? '<cols>' + cols + '</cols>' : '') + '<sheetData>' + body + '</sheetData></worksheet>' });
    });
    return zipStore(files);
  }

  /* ---------- 保存 ---------- */
  function download(blobOrText, filename, type) {
    var blob = blobOrText instanceof Blob ? blobOrText : new Blob([blobOrText], { type: type || 'text/plain;charset=utf-8' });
    var a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = filename;
    document.body.appendChild(a); a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  }
  function stamp() { var d = new Date(); function p(n) { return ('0' + n).slice(-2); } return '' + d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()); }

  /* ---------- ツール間の受け渡し ----------
   * 封筒（envelope）：{ scmExchange: 1, kind, from, title, created, payload }
   *  kind: 'cld' | 'svn' | 'workbench' | 'arrows' | 'series' | 'dataset' | 'hypotheses'
   * 同じブラウザで開いたツール同士は localStorage の受け箱（scm.inbox.<tool>）で渡す。
   * file:// で localStorage が分かれるブラウザでは，ファイル（.scm.json）で渡す。
   */
  var INBOX = 'scm.inbox.';
  function envelope(kind, from, payload, title) { return { scmExchange: 1, kind: kind, from: from, title: title || '', created: new Date().toISOString(), payload: payload }; }
  function isEnvelope(o) { return o && typeof o === 'object' && o.scmExchange === 1 && typeof o.kind === 'string'; }
  function post(toTool, env) {
    try { localStorage.setItem(INBOX + toTool, JSON.stringify(env)); return true; } catch (e) { return false; }
  }
  function take(tool) {
    try { var s = localStorage.getItem(INBOX + tool); if (!s) return null; localStorage.removeItem(INBOX + tool); var o = JSON.parse(s); return isEnvelope(o) ? o : null; } catch (e) { return null; }
  }
  function peek(tool) { try { var s = localStorage.getItem(INBOX + tool); return s ? JSON.parse(s) : null; } catch (e) { return null; } }
  /* ツールのファイル名：build.py が tools/ の実際のファイル名を TOOL_FILES に書き込む */
  var TOOL_FILES = /*TOOL_FILES*/{}/*END*/;
  var TOOL_LABEL = { index: '演習ファイル索引', navi: 'システム分析ナビ', workbench: 'システム分析ワークベンチ', lab: 'システム分析ラボ', data: 'データブラウザ' };
  var SELF = null;
  function rememberSelf(tool) {
    SELF = tool;
    try { var reg = JSON.parse(localStorage.getItem('scm.tools') || '{}'); reg[tool] = { href: location.href.split('#')[0], seen: new Date().toISOString() }; localStorage.setItem('scm.tools', JSON.stringify(reg)); } catch (e) { }
    // 開いてくれた相手（window.opener）に「準備できた」と知らせる
    try { if (window.opener && window.opener !== window) window.opener.postMessage({ scmReady: tool }, '*'); } catch (e) { }
  }
  function toolHref(tool) {
    // TOOL_FILES は tools/ フォルダから見た相対パス（索引だけは ../）。いまのページの場所から解決する
    var here = location.href.split('#')[0].split('?')[0], file = decodeURIComponent(here.replace(/^.*\//, ''));
    var base = here;
    if (TOOL_FILES.index && TOOL_FILES.index.replace(/^\.\.\//, '') === file && TOOL_FILES.index.indexOf('../') === 0) base = here.replace(/[^\/]*$/, '') + 'tools/x';
    if (TOOL_FILES[tool]) { try { return new URL(TOOL_FILES[tool].split('/').map(encodeURIComponent).join('/').replace(/^\.\./, '..'), base).href; } catch (e) { } }
    try { var reg = JSON.parse(localStorage.getItem('scm.tools') || '{}'); if (reg[tool]) return reg[tool].href; } catch (e) { }
    return null;
  }
  function openTool(tool, hash) {
    var href = toolHref(tool); if (!href) return null;
    return window.open(href + (hash ? '#' + hash : ''), 'scm_' + tool);
  }
  /* 渡して開く
   *  1) 受け箱（localStorage）に入れる（同じブラウザ・同じ場所のツールならこれで届く）
   *  2) 相手を開き，「準備できた」の合図を待って postMessage で渡す（file:// でも届く）
   *  3) どちらも確かめられなければ，.scm.json として保存し，相手の「開く」で読み込んでもらう
   *  cb(result) : 'delivered' | 'downloaded'
   */
  function sendTo(toTool, env, opts) {
    opts = opts || {};
    env.to = toTool; env.id = env.id || (Date.now().toString(36) + Math.random().toString(36).slice(2, 7));
    post(toTool, env);
    var w = openTool(toTool), done = false;
    function finish(r) { if (done) return; done = true; window.removeEventListener('message', onMsg); if (opts.done) opts.done(r); }
    function onMsg(ev) {
      var d = ev.data;
      if (w && ev.source === w && d && d.scmReady === toTool) { try { w.postMessage(env, '*'); } catch (e) { } }
      if (w && ev.source === w && d && d.scmAck === env.id) finish('delivered');
    }
    if (w) { window.addEventListener('message', onMsg); try { w.postMessage(env, '*'); } catch (e) { } }
    setTimeout(function () {
      if (done) return;
      download(JSON.stringify(env, null, 1), (opts.filename || (stamp() + '_scm-recurrent-program_' + TOOL_LABEL[toTool] + '用-' + env.kind)) + '.scm.json', 'application/json');
      finish('downloaded');
    }, opts.timeout || 6000);
    return w;
  }
  /* 受け取る側：起動時の受け箱と，postMessage の両方を見る。同じ封筒は1度だけ渡す */
  function receive(tool, cb) {
    var seen = {};
    function give(o, src) {
      if (!isEnvelope(o) || (o.to && o.to !== tool)) return;
      if (src) { try { src.postMessage({ scmAck: o.id }, '*'); } catch (e) { } }
      if (o.id && seen[o.id]) return; if (o.id) seen[o.id] = 1;
      try { localStorage.removeItem(INBOX + tool); } catch (e) { }
      cb(o);
    }
    window.addEventListener('message', function (ev) { give(ev.data, ev.source); });
    window.addEventListener('storage', function (ev) { if (ev.key === INBOX + tool && ev.newValue) { try { give(JSON.parse(ev.newValue)); } catch (e) { } } });
    var first = peek(tool); if (first) setTimeout(function () { give(first); }, 0);
    rememberSelf(tool);
  }
  /* ファイルから封筒を読む（.scm.json）：開くボタンの前段で使う */
  function envelopeFromText(text) { try { var o = JSON.parse(text); return isEnvelope(o) ? o : null; } catch (e) { return null; } }

  root.SCMIO = {
    version: VERSION,
    readFile: readFile, readXlsx: readXlsx, parseCsv: parseCsv, decodeText: decodeText,
    toObjects: toObjects, findHeaderRow: findHeaderRow, guessMapping: guessMapping, toDate: toDate, toNum: toNum,
    writeXlsx: writeXlsx, toCsv: toCsv, download: download, stamp: stamp,
    envelope: envelope, isEnvelope: isEnvelope, envelopeFromText: envelopeFromText, post: post, take: take, peek: peek,
    sendTo: sendTo, receive: receive, openTool: openTool, rememberSelf: rememberSelf, toolHref: toolHref,
    TOOL_LABEL: TOOL_LABEL, TOOL_FILES: TOOL_FILES
  };
})(typeof window !== 'undefined' ? window : this);
