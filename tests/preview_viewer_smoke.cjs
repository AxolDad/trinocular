// Execute the generated offline viewer with a minimal DOM/canvas test harness.
// This checks runtime behavior; it is not a browser screenshot/visual QA claim.
// Usage: node tests/preview_viewer_smoke.cjs /path/to/preview/index.html
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2],'utf8');
const embedded=html.match(/<script id="preview-data" type="application\/json">([\s\S]*?)<\/script>/)[1];
const code=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
let triangles=0,download=null,downloadName=null;
const events=new Map();
function element(id){return{id,value:'',checked:true,textContent:'',dataset:{},
 addEventListener(name,fn){events.set(id+':'+name,fn)}}}
const ids={};
for(const id of ['view','preview-data','angles','selected','badge','mode','edges','zoom','accept','note','status'])ids[id]=element(id);
ids['preview-data'].textContent=embedded;ids.mode.value='coverage';
ids.view.getBoundingClientRect=()=>({width:900,height:600});
ids.view.setPointerCapture=()=>{};
ids.view.getContext=()=>new Proxy({}, {get(target,key){if(key==='fill')return()=>triangles++;return()=>{}},set(){return true}});
const buttons=['front','back','left','right','top','bottom'].map(name=>{const b=element(name);b.dataset.pose=name;return b});
const context={console,Math,JSON,Blob,devicePixelRatio:1,
 requestAnimationFrame:fn=>fn(),setTimeout:fn=>fn(),addEventListener:()=>{},
 URL:{createObjectURL(blob){download=blob;return'blob:test'},revokeObjectURL(){}},
 document:{getElementById:id=>ids[id],querySelectorAll:()=>buttons,
 createElement:()=>({set download(value){downloadName=value},set href(value){},click(){}})}};
vm.runInNewContext(code,context,{filename:'generated-preview-viewer.js'});
assert.ok(triangles>100,'initial mesh must be drawn');
const initial=triangles;
events.get('back:click')();assert.ok(triangles>initial,'view buttons redraw the mesh');
events.get('view:pointerdown')({clientX:10,clientY:10,pointerId:1});
events.get('view:pointermove')({clientX:25,clientY:35});events.get('view:pointerup')();
events.get('accept:click')();assert.equal(download,null,'blank review cannot approve coverage');
ids.note.value='Synthetic software smoke test: view controls and review export checked';
events.get('accept:click')();assert.ok(download);assert.equal(downloadName,'review.json');
download.text().then(text=>{
 const review=JSON.parse(text),data=JSON.parse(embedded);
 assert.equal(review.decision,'approved');
 assert.equal(review.dataset_fingerprint,data.report.dataset_fingerprint);
 assert.equal(review.preview_report_sha256,data.preview_report_sha256);
 console.log('Viewer runtime smoke test passed: drawing, rotation, review validation, and export');
}).catch(error=>{console.error(error);process.exitCode=1});
