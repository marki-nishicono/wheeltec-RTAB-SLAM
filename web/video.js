'use strict';
// Length-prefixed Annex-B H.264 access units, decoded entirely on this machine.
(function(){
 const canvas=document.getElementById('camera'),ctx=canvas.getContext('2d'),note=document.getElementById('videoStatus');
 const button=document.getElementById('openVideo');
 let decoded=0,enabled=false,running=false,currentAbort=null;
 function close(){enabled=false;currentAbort?.abort();button.textContent='开启相机画面';note.textContent='画面已关闭 · 不编码、不推流';ctx.clearRect(0,0,canvas.width,canvas.height)}
 button.onclick=()=>{if(enabled)close();else{enabled=true;button.textContent='关闭相机画面';note.textContent='正在连接相机，等待首帧（最多 5 秒）…';if(!running)run()}};
 window.addEventListener('pagehide',close);
 document.addEventListener('visibilitychange',()=>{if(document.hidden)close()});
 if(!window.VideoDecoder){button.disabled=true;note.textContent='当前浏览器不支持 WebCodecs，请用新版 Edge / Chrome 打开 http://127.0.0.1:8080/';return}
 function codecFrom(data){
  for(let i=0;i<data.length-7;i++){
   let j=data[i]===0&&data[i+1]===0&&data[i+2]===1?i+3:data[i]===0&&data[i+1]===0&&data[i+2]===0&&data[i+3]===1?i+4:-1;
   if(j>=0&&(data[j]&31)===7)return 'avc1.'+[data[j+1],data[j+2],data[j+3]].map(v=>v.toString(16).padStart(2,'0')).join('');
  }return null;
 }
 async function run(){
 running=true;
 while(enabled){
  let decoder,reader,abort=new AbortController(),timer=setTimeout(()=>abort.abort(),5000);
  currentAbort=abort;
  try{
   decoder=new VideoDecoder({output:frame=>{if(enabled){canvas.width=frame.displayWidth;canvas.height=frame.displayHeight;ctx.drawImage(frame,0,0);decoded++;note.dataset.decoded=decoded}frame.close();},error:e=>{if(enabled)note.textContent='解码重连：'+e.message;abort.abort()}});
   const response=await fetch('/api/video.h264',{signal:abort.signal,cache:'no-store'});
   if(!response.ok)throw Error(response.status===502?'车辆未连接或服务未启动':response.status===503?'相机无新图像，请检查相机连接和采集服务':'视频服务 '+response.status);reader=response.body.getReader();
   let pending=new Uint8Array(),configured=false,waitingKey=true,base=null;
   while(true){
    const {value,done}=await reader.read();if(done)throw Error('视频流结束');
    clearTimeout(timer);timer=setTimeout(()=>abort.abort(),4000);
    const joined=new Uint8Array(pending.length+value.length);joined.set(pending);joined.set(value,pending.length);pending=joined;
    let offset=0;
    while(pending.length-offset>=13){
     const header=new DataView(pending.buffer,pending.byteOffset+offset),size=header.getUint32(0);
     if(size>2000000)throw Error('视频帧长度异常');if(pending.length-offset<13+size)break;
     const key=header.getUint8(4)!==0,stamp=Number(header.getBigUint64(5)),data=pending.slice(offset+13,offset+13+size);offset+=13+size;
     if(!configured){
      if(!key)continue;const codec=codecFrom(data);if(!codec)continue;
      let config={codec,optimizeForLatency:true,hardwareAcceleration:'prefer-hardware'};
      let support=await VideoDecoder.isConfigSupported(config);
      if(!support.supported){config.hardwareAcceleration='no-preference';support=await VideoDecoder.isConfigSupported(config)}
      if(!support.supported)throw Error('浏览器不支持 '+codec);
      decoder.configure(config);configured=true;
      note.textContent='H.264 · 本机解码 · '+(config.hardwareAcceleration==='prefer-hardware'?'请求 GPU 优先（实际由浏览器选择）':'浏览器自动选择解码器');
     }
     if(decoder.decodeQueueSize>3){decoder.reset();configured=false;waitingKey=true;continue}
     if(waitingKey&&!key)continue;waitingKey=false;if(base===null)base=stamp;
     decoder.decode(new EncodedVideoChunk({type:key?'key':'delta',timestamp:stamp-base,data}));
    }pending=pending.slice(offset);
   }
  }catch(e){if(enabled)note.textContent='暂无画面 · '+(e.name==='AbortError'?'等待图像超时':e.message)+' · 正在重试，可点击关闭'}
  finally{clearTimeout(timer);abort.abort();if(reader)try{await reader.cancel()}catch{}if(decoder&&decoder.state!=='closed')decoder.close()}
  if(enabled)await new Promise(r=>setTimeout(r,1000));
 }
 running=false;currentAbort=null;
 }
})();
