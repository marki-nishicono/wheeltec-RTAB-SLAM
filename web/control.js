'use strict';
class DriveChannel{
 constructor(onState,onLost){this.onState=onState;this.onLost=onLost;this.socket=null;this.seq=0;this.connect()}
 connect(){
  const ws=new WebSocket('ws://'+location.hostname+':8081');this.socket=ws;
  ws.onmessage=e=>{try{this.onState(JSON.parse(e.data))}catch(err){console.error(err)}};
  ws.onclose=()=>{this.onLost();setTimeout(()=>this.connect(),1000)};ws.onerror=()=>ws.close();
 }
 send(speed,steer,token,emergency=false,epoch=0){
  const ws=this.socket;if(!ws||ws.readyState!==WebSocket.OPEN)return false;
  if(ws.bufferedAmount>2048){ws.close();return false}
  ws.send(JSON.stringify({seq:++this.seq,speed,steer,token,emergency,epoch}));return true;
 }
 ping(){if(this.socket?.readyState===WebSocket.OPEN&&this.socket.bufferedAmount<2048)this.socket.send(JSON.stringify({type:'ping',stamp:performance.now()}))}
}
