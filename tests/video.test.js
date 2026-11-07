// No browser network, GPU, or vehicle required.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
let requests=0,signal;const items={camera:{getContext:()=>({clearRect(){}})},videoStatus:{},openVideo:{}};
const context={document:{getElementById:id=>items[id],addEventListener(){}},window:{VideoDecoder:true,addEventListener(){}},
 AbortController,setTimeout,clearTimeout,VideoDecoder:class{constructor(){}close(){}},
 fetch:(_url,options)=>{requests++;signal=options.signal;return new Promise((resolve,reject)=>signal.addEventListener('abort',()=>reject(Error('aborted'))))}};
vm.runInNewContext(fs.readFileSync('web/video.js','utf8'),context);
assert.equal(requests,0);items.openVideo.onclick();assert.equal(requests,1);
items.openVideo.onclick();assert.equal(signal.aborted,true);
setTimeout(()=>{assert.equal(requests,1);console.log('Video off by default, opt-in, abort and no restart after off: PASS')},1100);
