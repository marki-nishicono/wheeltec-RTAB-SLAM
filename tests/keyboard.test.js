// Run isolated keyboard behavior with no browser or vehicle connection.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync('web/app.js','utf8');let listeners={},commands=[];
const arm={checked:true};
const context={keys:new Set(),moving:null,lastControl:1000,performance:{now:()=>1100},document:{hidden:false},
 controlState:{ages:{control:.1,wheel:.1,scan_raw:.1},control:{enabled:true}},
 $:id=>arm,drive:(...x)=>commands.push(x),window:{addEventListener:(name,f)=>listeners[name]=f}};
context.stop=()=>{context.moving=null;context.keys.clear();arm.checked=false;commands.push([0,0])};
vm.createContext(context);
vm.runInContext(source.slice(source.indexOf('function controlBlock()'),source.indexOf('function renderControl()')),context);
vm.runInContext(source.slice(source.indexOf('function keyboardDrive()'),source.indexOf("document.querySelectorAll('[data-v]')")),context);
function event(code){return {code,target:{tagName:'BODY'},preventDefault(){}}}
listeners.keydown(event('KeyW'));assert.deepEqual(commands.at(-1),[.10,0]);
listeners.keydown(event('KeyA'));assert.deepEqual(commands.at(-1),[.10,.3]);
listeners.keyup(event('KeyW'));assert.deepEqual(commands.at(-1),[0,0]);
listeners.keydown(event('KeyS'));assert.deepEqual(commands.at(-1),[-.10,.3]);
listeners.keydown(event('Space'));assert.deepEqual(commands.at(-1),[0,0]);assert.equal(arm.checked,false);
let count=commands.length;listeners.keydown(event('KeyW'));assert.equal(commands.length,count);
arm.checked=true;context.lastControl=0;listeners.keydown(event('KeyW'));assert.equal(commands.length,count);
context.lastControl=1000;context.controlState.ages.control=20;listeners.keydown(event('KeyW'));assert.equal(commands.length,count);
context.controlState.ages.control=.1;context.controlState.ages.wheel=1;listeners.keydown(event('KeyW'));assert.equal(commands.length,count);
context.controlState.ages.wheel=.1;context.controlState.drive_block='imu_calibrating_keep_stationary';listeners.keydown(event('KeyW'));assert.equal(commands.length,count);
console.log('WASD, reverse steering, release, space disarm, lock and stale state: PASS');
