'use strict';
if(location.protocol==='file:'){
 document.body.replaceChildren();
 const p=document.createElement('p');p.textContent='实验台需要通过本机服务打开，正在转到 http://127.0.0.1:8080/';
 const a=document.createElement('a');a.href='http://127.0.0.1:8080/';a.textContent='进入实验台';document.body.append(p,a);
 location.replace(a.href);
}else{
 (async()=>{for(const name of ['control.js','app.js','video.js'])await new Promise((resolve,reject)=>{
  const s=document.createElement('script');s.src=name;s.onload=resolve;s.onerror=()=>reject(Error(name+' 加载失败，请刷新页面'));document.body.append(s);
 })})().catch(e=>{document.getElementById('status').textContent=e.message});
}
