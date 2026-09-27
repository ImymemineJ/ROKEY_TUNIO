const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(__dirname + '/../web/app.js', 'utf8');
const start = source.indexOf('function renderButtons(state)');
const end = source.indexOf('function renderSettingsLock', start);
const pending = new Map();
const buttons = new Map();
const context = {
  commandPending:false, streamReady:true, lastState:{phase:2}, PHASE:{PAUSED:9},
  registration:{busy:false,message:''}, isRunning:()=>true,
  $:id=>{if (!buttons.has(id)) buttons.set(id,{});return buttons.get(id);},
  fetch:url=>new Promise(resolve=>pending.set(url,resolve)), toast:()=>{},
};
vm.createContext(context);
vm.runInContext(source.slice(start,end),context);
(async()=>{
  const pause = context.sendCommand('pause');
  assert.equal(context.commandPending,'pause');
  assert.equal(buttons.get('btn-cancel').disabled,false);
  const cancel = context.sendCommand('cancel');
  assert.equal(context.commandPending,'cancel');
  assert.equal(buttons.get('btn-cancel').disabled,true);
  pending.get('/api/command/pause')({ok:true,json:async()=>({accepted:true})});
  await pause;
  assert.equal(context.commandPending,'cancel');
  pending.get('/api/command/cancel')({ok:true,json:async()=>({accepted:true})});
  await cancel;
  assert.equal(context.commandPending,false);
  console.log('일시정지 응답 대기 중 취소·늦은 응답 경합 통과');
})();
