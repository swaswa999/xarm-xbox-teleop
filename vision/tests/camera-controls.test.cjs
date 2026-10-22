const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../live.html'),'utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const settle=()=>new Promise(resolve=>setImmediate(resolve));
function element(){
  const item={value:'',children:[],style:{},classList:{toggle(){}},videoWidth:1280,videoHeight:720,
    textContent:'',disabled:false,hidden:false,srcObject:null,paused:true,
    addEventListener(){},removeAttribute(){},pause(){this.paused=true;},async play(){this.paused=false;},
    getContext(){return new Proxy({},{get:()=>()=>{}});},
    replaceChildren(...children){this.children=children;this.value=children[0]?.value||'';},append(x){this.children.push(x);},click(){}};
  return item;
}
function mediaStream(id,label){
  const listeners={};
  const track={label,stopped:0,stop(){this.stopped++;},getSettings(){return{deviceId:id};},addEventListener(name,fn){listeners[name]=fn;},emit(name){listeners[name]?.();}};
  return{track,getTracks:()=>[track],getVideoTracks:()=>[track]};
}
function setup(){
  const nodes=new Map();
  const state={devices:[{kind:'videoinput',deviceId:'built-in',label:'Mac webcam'},{kind:'videoinput',deviceId:'usb',label:'USB camera'},{kind:'audioinput',deviceId:'mic',label:'Microphone'}],streams:[],calls:[],listeners:{}};
  const mediaDevices={async enumerateDevices(){return state.devices;},async getUserMedia(options){state.calls.push(options);const id=options.video.deviceId?.exact||'built-in';const d=state.devices.find(d=>d.deviceId===id);const stream=mediaStream(id,d?.label||'Camera');state.streams.push(stream);return stream;},addEventListener(name,fn){state.listeners[name]=fn;}};
  const context=vm.createContext({console,performance,URL,Blob,setTimeout,structuredClone,requestAnimationFrame(){},navigator:{mediaDevices},location:{host:'localhost:8766'},window:{addEventListener(){}},Option:function(text,value){this.text=text;this.value=value;},document:{getElementById(id){if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);},createElement:element}});
  vm.runInContext(script,context);
  return{state,nodes,mediaDevices,run:code=>vm.runInContext(code,context)};
}
test('connects selected camera and switches during tracking, releasing the previous camera',async()=>{
  const t=setup();await settle();
  await t.run("camera('built-in')");
  assert.equal(t.state.calls[0].video.deviceId.exact,'built-in');
  assert.equal(t.nodes.get('devices').disabled,false);
  t.run("mode='tracking';boxes=[{id:'T1',label:'tire'}];ws={close(){}};");
  await t.run("camera('usb')");
  assert.equal(t.state.calls[1].video.deviceId.exact,'usb');
  assert.equal(t.state.streams[0].track.stopped,1);
  assert.equal(t.nodes.get('devices').value,'usb');
  assert.equal(t.nodes.get('camera-active').textContent,'Active: USB camera');
  assert.equal(t.run('mode'),'preview');
  assert.equal(t.run('boxes.length'),0);
});
test('hotplug refresh includes new cameras and preserves the active selection',async()=>{
  const t=setup();await settle();await t.run("camera('usb')");
  t.state.devices.push({kind:'videoinput',deviceId:'usb2',label:'Second USB camera'});
  t.state.listeners.devicechange();await settle();
  assert.equal(t.nodes.get('devices').children.length,3);
  assert.equal(t.nodes.get('devices').value,'usb');
  t.state.devices=t.state.devices.filter(d=>d.deviceId!=='usb');
  t.state.streams[0].track.emit('ended');await settle();
  assert.equal(t.run('mode'),'offline');
  assert.equal(t.nodes.get('devices').children.length,2);
  assert.match(t.nodes.get('status').textContent,/disconnected/);
});
test('permission failure leaves controls usable for retry',async()=>{
  const t=setup();await settle();
  t.mediaDevices.getUserMedia=async()=>{throw Object.assign(new Error('Denied'),{name:'NotAllowedError'});};
  await t.run("camera('usb')");await settle();
  assert.equal(t.run('mode'),'offline');
  assert.equal(t.nodes.get('camera-start').disabled,false);
  assert.match(t.nodes.get('status').textContent,/denied/);
});
test('cancelling pending camera access stops any stream that arrives late',async()=>{
  const t=setup();await settle();let complete;
  t.mediaDevices.getUserMedia=()=>new Promise(resolve=>{complete=resolve;});
  const pending=t.run("camera('usb')");
  t.run('disconnect()');
  const late=mediaStream('usb','USB camera');complete(late);await pending;
  assert.equal(late.track.stopped,1);
  assert.equal(t.run('mode'),'offline');
  assert.equal(t.nodes.get('source').srcObject,null);
});
