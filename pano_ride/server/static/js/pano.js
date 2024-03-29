var trajs=[]
var texture_cache=[]
var mymap
var camera, scene, renderer
var camera_map, scene_map, renderer_map
var graph={"nodes":[],"edges":[]}
var isUserInteracting = false,
    lon = 0, lat = 0,
    distance = 1.1,
    onPointerDownPointerX = 0,
    onPointerDownPointerY = 0,
    onPointerDownLon = 0,
    onPointerDownLat = 0;
var scene_scale=1
var scene_yaw=0
var scene_cur_pitch=0.1
var scene_start_pitch=scene_cur_pitch
var lock_status_scene=0
var onPointerDownPitch = 0
var last_scene_x=0
var isUserInteractingScene=false
var pano_material
var sence_cam_dis=-500
var cur_node=null
var next_node=null
var traj_threejs=[]
var up_key=false
var right_key=false
var left_key=false
var back_key=false
var me=null
var voice_list=[]
var mp3_list={}
var lm_mp3_files={}
var lm_jpg_files={}
var landmarks={}
var cur_voice={"name":"","dom":null}
var moving_speed=1
var node_threejs=[]
var raycaster
var lm_sprit_list=[]
var bgm_file=null
var bgm_dom=null
var res_tex_list={}
var lm_icon_filename="lmicon.jpg"
var lm_icon_filename_a="lmicon_a.jpg"
var soundBGM

function show_img_callback(node) {
    return (texture) => {
//        texture.generateMipmaps = false;
//        texture.wrapS = texture.wrapT = THREE.ClampToEdgeWrapping;
//        texture.minFilter = THREE.LinearFilter;
        pano_material.map=texture
        pano_material.map.needsUpdate=true
        cur_node=node
    }
}

function load_img_res_callback(res_name){
    return (texture) => {
            res_tex_list[res_name]=texture
        }
}

function show_pc(mp_file){
    var fr = new FileReader();
    fr.onload = function(e) {
        var lines = e.target.result;
        var mps = JSON.parse(lines)
        var all_mps=[]
        for(var frame_id in mps){
            sub_mps=mps[frame_id]
            for (var i=0; i<sub_mps.length; i++){
                all_mps.push(sub_mps[i])
            }
        }
        var color="#ffffff"
        add_threejs_pts(all_mps, color, 1)
    };
    fr.readAsText(mp_file);
}

function add_threejs_pts(pts, color, size){
    var pointMaterial =  new THREE.PointsMaterial({
        size: size,
        color: color,
        sizeAttenuation: false
    });
    var tmp_geo = new THREE.Geometry();
    var tmp_mat = pointMaterial;
    for (var i=0; i<pts.length; i++) {
        tmp_geo.vertices.push(new THREE.Vector3(pts[i][0], pts[i][2], -pts[i][1]));
    }
    var tmp_threejs_pts = new THREE.Points(tmp_geo, tmp_mat);
    traj_threejs.push(tmp_threejs_pts)
    scene_map.add(tmp_threejs_pts);
    return tmp_threejs_pts
}

function add_threejs_lines(pts,color){
    var points = []
    for (var i=0; i<pts.length; i++){
        points.push( new THREE.Vector3(pts[i][0], pts[i][2], -pts[i][1]))
    }
    var trajGeometry = new THREE.Geometry()
    trajGeometry.setFromPoints( points )
    var trajLine = new THREE.Line( trajGeometry, new THREE.LineBasicMaterial({
        color: color, linewidth: 1, opacity: 1
    }));
    traj_threejs.push(trajLine)
    scene_map.add(trajLine);
    return trajLine
}

function add_threejs_segment(segs,color, addlist=true){
    points=[]
    for (var i=0; i<segs.length; i++){
        points.push( new THREE.Vector3(segs[i][0][0], segs[i][0][2], -segs[i][0][1]))
        points.push( new THREE.Vector3(segs[i][1][0], segs[i][1][2], -segs[i][1][1]))
    }
    var arrGeometry = new THREE.Geometry()
    arrGeometry.setFromPoints( points )
    var lineobj = new THREE.LineSegments( arrGeometry, new THREE.LineBasicMaterial({
        color: color, linewidth: 1, opacity: 1
    }));
    if (addlist){
        traj_threejs.push(lineobj)
    }
    scene_map.add(lineobj)
    return lineobj
}
function show_graph_3d(){
    var tmp_pts=[]
    for (var i=0; i<graph["edges"].length; i++){
        var p1=graph["edges"][i][0]["posi"]
        var p2=graph["edges"][i][1]["posi"]
        tmp_pts.push([p1,p2])
    }
    add_threejs_segment(tmp_pts,"#ff5522")
    var tmp_pts=[]
    for (var i=0; i<graph["nodes"].length; i++){
        var p1=graph["nodes"][i]["posi"]
        tmp_pts.push(p1)
    }
    node_threejs.push(add_threejs_pts(tmp_pts,"#ff5522",2))
    tmp_pts=[]
    tmp_pts.push([0,0,0])
    me = add_threejs_pts(tmp_pts,"#55eeff",10)
}

function init_scene() {
    var container,container_query, mesh;
    container = document.getElementById( 'scene' );
    container_query=$("#scene");
    var divWidth = container_query.width();
    var divHeight = container_query.height();
    camera_map = new THREE.OrthographicCamera( divWidth/-2, divWidth/2, divHeight/2, divHeight/-2, 0.1, 10000 )
    camera_map.zoom=150
    camera_map.up.x = 0;
    camera_map.up.y = 0;
    camera_map.up.z = 1;
    camera_map.aspect = divWidth/divHeight;
    camera_map.updateProjectionMatrix();
    scene_map = new THREE.Scene();
    renderer_map = new THREE.WebGLRenderer();
    renderer_map.setPixelRatio( window.devicePixelRatio );
    renderer_map.setSize( divWidth, divHeight);
    container.appendChild( renderer_map.domElement );
    container_query.on( 'mousedown', onSceneMouseDown);
    container_query.on( 'mousemove', onSceneMouseMove);
    container_query.on( 'mouseup', onSceneMouseUp);
    container_query.on( 'mouseout', onSceneMouseOut);
    renderer_map.domElement.addEventListener("click", onclick, true);
    raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.03
    raycaster.linePrecision = 0.03
}

function onclick(event) {
    event.preventDefault();
    var mouse = new THREE.Vector2()
    var container_query=$("#scene");
    var wWidth = window.innerWidth;
    var wHeight = window.innerHeight;
    var divWidth = container_query.width();
    var divHeight = container_query.height();
    mouse.x = ((event.clientX-(wWidth-divWidth))/divWidth)*2-1;
    mouse.y = - ( event.clientY / container_query.height() ) * 2 + 1;
    raycaster.setFromCamera(mouse, camera_map)
    if (node_threejs.length>0){
        var intersects = raycaster.intersectObjects(node_threejs, true)
        if (intersects.length > 0) {
            var pt_ind=intersects[0].index
            next_node=graph["nodes"][pt_ind]
        }
    }
}

function init_pano() {
    var container,container_query, mesh;
    container = document.getElementById( 'container' );
    container_query=$("#container");
    var divHeight = window.innerHeight;
    var divWidth = window.innerWidth;
    var rate=divWidth/divHeight
    camera = new THREE.PerspectiveCamera( 60, rate, 1, 3000 );
    camera.target = new THREE.Vector3( 0, 0, 0 );
    scene = new THREE.Scene();
    var geometry = new THREE.SphereBufferGeometry( 1100, 60, 40 );
    // invert the geometry on the x-axis so that all of the faces point inward
    geometry.scale( -1, 1, 1 );
    var texture =  new THREE.TextureLoader().load('#');
    pano_material = new THREE.MeshBasicMaterial( { map: texture } );
    mesh = new THREE.Mesh( geometry, pano_material );
    scene.add( mesh );
    renderer = new THREE.WebGLRenderer();
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( divWidth, divHeight);
    container.appendChild( renderer.domElement );
    container_query.on( 'mousedown', onDocumentMouseDown);
    container_query.on( 'mousemove', onDocumentMouseMove);
    container_query.on( 'mouseup', onDocumentMouseUp);
    container_query.on( 'mouseout', onDocumentMouseOut);
    renderer.domElement.addEventListener("click", onclickpano, true);
}

function onSceneMouseDown( event ) {
    event.preventDefault()
    event.stopPropagation()
    isUserInteractingScene = true;
    last_scene_x = event.clientX;
    onPointerDownPointerY = event.clientY;
    onPointerDownPitch=scene_cur_pitch;
    lock_status_scene=0
}
function onSceneMouseMove( event ) {
    if ( isUserInteractingScene === true ) {
        if (lock_status_scene==0){
            if (Math.abs(event.clientX-last_scene_x)>10){
                last_scene_x=event.clientX
                lock_status_scene=1
            }
            if (Math.abs(event.clientY-onPointerDownPointerY)>10){
                onPointerDownPointerY = event.clientY;
                lock_status_scene=-1
            }
        }
        if (lock_status_scene==1){
            if (event.clientX-last_scene_x>1){
                camera_map.zoom = camera_map.zoom* 1.3
                if (camera_map.zoom>200){
                    camera_map.zoom=200
                }
                camera_map.updateProjectionMatrix();
            }else if(event.clientX-last_scene_x<-1){
                camera_map.zoom = camera_map.zoom* 0.9
                if (camera_map.zoom<20){
                    camera_map.zoom=20
                }
                camera_map.updateProjectionMatrix();
            }
            last_scene_x=event.clientX
        }
        if (lock_status_scene==-1){
            last_scene_x=event.clientX
            scene_cur_pitch = ( event.clientY - onPointerDownPointerY ) * -0.8 + onPointerDownPitch;
            if (scene_cur_pitch>90){
                onPointerDownPointerY=onPointerDownPointerY-(scene_cur_pitch-90)
                scene_cur_pitch=90
            }
            if (scene_cur_pitch<0.1){
                onPointerDownPointerY=onPointerDownPointerY+(0.1-scene_cur_pitch)
                scene_cur_pitch=0.1
            }
        }
    }
}
function onSceneMouseOut(){
    isUserInteractingScene=false
}
function onSceneMouseUp() {
    isUserInteractingScene = false;
}
function onDocumentMouseOut(){
    isUserInteracting=false
}
function onDocumentMouseDown( event ) {
    event.preventDefault();
    isUserInteracting = true;
    onPointerDownPointerX = event.clientX;
    onPointerDownPointerY = event.clientY;
    onPointerDownLon = lon;
    onPointerDownLat = lat;
}
function onDocumentMouseMove( event ) {
    if ( isUserInteracting === true ) {
        lon = ( onPointerDownPointerX - event.clientX ) * 0.08 + onPointerDownLon;
        lat = ( event.clientY - onPointerDownPointerY ) * -0.08 + onPointerDownLat;
        if (lat>9){
            onPointerDownPointerY=onPointerDownPointerY-(lat-9)
            lat=9
        }
        if (lat<-27){
            onPointerDownPointerY=onPointerDownPointerY+(-27-lat)
            lat=-27
        }
    }
}
function onDocumentMouseUp() {
    isUserInteracting = false;
}
function animate() {
    requestAnimationFrame( animate );
    update();
}
function update() {
    if (cur_node==null){
        return
    }
    var phi = THREE.Math.degToRad( 90 - lat );
    var pano_theta = THREE.Math.degToRad( lon-cur_node["dir"]);
    camera.position.x = distance * Math.sin( phi ) * Math.cos( pano_theta );
    camera.position.y = distance * Math.cos( phi );
    camera.position.z = distance * Math.sin( phi ) * Math.sin( pano_theta );
    camera.lookAt( camera.target );
    update_scene_map();
    renderer.render( scene, camera );
    renderer_map.render( scene_map, camera_map );
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function get_command(){
    if (up_key && !right_key && !back_key && !left_key){
        return 0
    }
    if (up_key && right_key && !back_key && !left_key){
        return 45
    }
    if (!up_key && right_key && !back_key && !left_key){
        return 90
    }
    if (!up_key && right_key && back_key && !left_key){
        return 135
    }
    if (!up_key && !right_key && back_key && !left_key){
        return 180
    }
    if (up_key && !right_key && !back_key && left_key){
        return -45
    }
    if (!up_key && !right_key && !back_key && left_key){
        return -90
    }
    if (!up_key && !right_key && back_key && left_key){
        return -135
    }
    return 9999
}

function show_image(node){
    var reader = new FileReader();
    reader.onload = function (e) {
        var loader = new THREE.TextureLoader();
        loader.load(e.target.result, show_img_callback(node));
    };
    reader.readAsDataURL(node["f"]);
}

function update_scene_map(){
    if (cur_node==null){
        return
    }
    var p_tmp=cur_node["posi"]
    var temp_pitch_rad = THREE.Math.degToRad( scene_cur_pitch );
    var temp_yaw_rad=THREE.Math.degToRad( lon)
    cam_x=sence_cam_dis * Math.sin( temp_pitch_rad ) * Math.sin(temp_yaw_rad)+p_tmp[0]
    cam_y=sence_cam_dis * Math.cos( temp_pitch_rad ) +p_tmp[1]
    cam_z=sence_cam_dis * Math.sin( temp_pitch_rad ) * Math.cos(temp_yaw_rad)+p_tmp[2]
    me.position.set(p_tmp[0], p_tmp[2], -p_tmp[1])
    camera_map.position.x = cam_x;
    camera_map.position.y = cam_z;
    camera_map.position.z = -cam_y;
    camera_map.lookAt( new THREE.Vector3( p_tmp[0], p_tmp[2], -p_tmp[1] ) );
}

function check_run(val){
    if (val){
        moving_speed=3
    }else{
        moving_speed=1
    }
}

async function play_thread(file_list){
    while (true){
        if (cur_node["id"]==null || next_node["id"]==null){
            await sleep(100)
        }
        if (cur_node["id"]!=next_node["id"]){
            show_image(next_node)
            update_scene_map()
            update_voice_list()
            update_landmarks()
        }
        var angle=get_command()
        if (angle>1000){
            await sleep(100)
        }else{
            var angle_world = angle+lon
            if (angle_world>180){
                angle_world=angle_world-360
            }else if(angle_world<-180){
                angle_world=angle_world+360
            }
            var min_angle=-1
            var min_id=-1
            for (var i=0; i<cur_node["c"].length; i++){
                var cam_angle_w = get_angle_from_mat(cur_node["posi"], cur_node["c"][i]["posi"])
                cam_angle_w=cam_angle_w*180/Math.PI
                var angle_diff = angle_world-cam_angle_w
                if (angle_diff>180){
                    angle_diff=angle_diff-360
                }else if(angle_diff<-180){
                    angle_diff=angle_diff+360
                }
                angle_diff = Math.abs(angle_diff)
                if(angle_diff<min_angle || min_angle==-1){
                    min_angle=angle_diff
                    min_id=i
                }
            }
            if (min_angle<60 && min_id>=0){
                next_node=cur_node["c"][min_id]
                tmp_dist=cal_dist(next_node["posi"], cur_node["posi"])
                wait_time=tmp_dist/moving_speed*1000
                await sleep(wait_time)
            }else{
                await sleep(100) 
            }
        }
    }
}

function get_angle_from_mat(ori, tar){
    var dir=[tar[2]-ori[2], tar[0]-ori[0]]
    var angle = Math.atan2( dir[1], dir[0] )
    return angle
}

function show_all_landmarks(){
    all_lms=[]
    for(var i=0;i<landmarks.length; i++){
        all_lms.push(landmarks[i]["posi"])
    }
    var color="#ffff00"
    add_threejs_pts(all_lms, color, 10)
}

function get_sub(v1,v2){
    return [v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2],]
}

function cal_pano_posi(tar_posi, me_posi, me_dir){
    rel_posi = get_sub(tar_posi,me_posi)
    norm = cal_dist(rel_posi,[0,0,0])
    rel_posi=[rel_posi[0]/norm*1000,rel_posi[1]/norm*1000,rel_posi[2]/norm*1000]
    rel_cam_posi=[-rel_posi[2],-rel_posi[1],-rel_posi[0]]
    return rel_cam_posi
}

function onclickpano(event){
    event.preventDefault();
    var mouse = new THREE.Vector2()
    mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
    mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
    raycaster.setFromCamera(mouse, camera)
    if (lm_sprit_list.length>0){
        var intersects = raycaster.intersectObjects(lm_sprit_list, true)
        if (intersects.length > 0) {
            var jpg_name=intersects[0].object.jpg
            var reader = new FileReader();
            reader.onload = function (e) {
                img_str='<img src='+e.target.result+' class="fit_div" align="right">'
                var lm_img_dom = document.getElementById( 'lm_img' );
                lm_img_dom.innerHTML=img_str
                check_lm_img(false)
            };
            reader.readAsDataURL(lm_jpg_files[jpg_name]);
            var mp3_name=intersects[0].object.mp3
            if (mp3_name ==cur_voice["name"]){
                if (cur_voice["dom"].paused){
                    cur_voice["dom"].play()
                }else{
                    cur_voice["dom"].pause()
                }
                return
            }
            if (cur_voice["name"]=="" || mp3_name !=cur_voice["name"]){
                if (cur_voice["name"]!=""){
                    cur_voice["dom"].pause()
                }
                var mp3_file=lm_mp3_files[mp3_name]
                var reader = new FileReader();
                reader.onload = function (e) {
                    var soundFile = document.createElement("audio")
                    soundFile.preload = "auto"
                    var src = document.createElement("source")
                    src.src = e.target.result
                    soundFile.appendChild(src)
                    soundFile.load()
                    soundFile.play()
                    cur_voice={"name":mp3_name, "dom":soundFile}
                };
                reader.readAsDataURL(mp3_file);
            }
        }
    }
}

function update_landmarks(){
    var display_list=[]
    for(var i=0;i<landmarks.length; i++){
        var range=landmarks[i]["range"]
        if (range[0]<cur_node["id"] && range[1]>cur_node["id"]){
            display_list.push(1)
        }else{
            display_list.push(0)
        }
    }
    
    for(var i=0; i<display_list.length; i++){
        var b_show_ind=-1
        for (var j=0; j<lm_sprit_list.length; j++){
            if(lm_sprit_list[j]["iddd"].valueOf()==landmarks[i]["id"].valueOf()){
                b_show_ind=j
                break
            }
        }
        if (display_list[i]==1){
            posi_3d=landmarks[i]["posi"]
            posi_pano = cal_pano_posi(posi_3d, cur_node["posi"], cur_node["dir"])
            if (b_show_ind==-1){
                if (!(lm_icon_filename in res_tex_list)){
                    console.log("marker icon not loaded")
                    continue
                }
                var spriteMap = res_tex_list[lm_icon_filename];
                var spriteMapAlpha = res_tex_list[lm_icon_filename_a];
                var spriteMaterial = new THREE.SpriteMaterial( { map: spriteMap, alphaMap: spriteMapAlpha } );
                spriteMaterial.sizeAttenuation=false
                var sprite = new THREE.Sprite( spriteMaterial );
                sprite.position.set(posi_pano[0],posi_pano[1],posi_pano[2]);
                sprite.center.set( 0.5, 0.5 );
                sprite.scale.set( .30, .30, .30 );
                sprite["mp3"]=landmarks[i]["mp3"]
                sprite["jpg"]=landmarks[i]["jpg"]
                sprite["iddd"]=landmarks[i]["id"]
                scene.add(sprite)
                lm_sprit_list.push(sprite)
            }else{
                lm_sprit_list[b_show_ind].position.set(posi_pano[0],posi_pano[1],posi_pano[2]);
            }
        }else{
            if (b_show_ind!=-1){
                scene.remove(lm_sprit_list[b_show_ind])
                var new_array=[]
                for (var j=0; j<lm_sprit_list.length; j++){
                    if(j!=b_show_ind){
                        new_array.push(lm_sprit_list[j])
                    }
                }
                lm_sprit_list=new_array
            }
        }
    }
}

function update_bgms(){
    if (bgm_file==null){
        return
    }
    var reader = new FileReader();
    reader.onload = function (e) {
        soundBGM = document.createElement("audio")
        soundBGM.preload = "auto"
        var src = document.createElement("source")
        src.src = e.target.result
        soundBGM.appendChild(src)
        soundBGM.load()
        soundBGM.loop=true
        soundBGM.play()
        bgm_dom=soundBGM
    };
    reader.readAsDataURL(bgm_file);
}

function process_folder(){
    var file_list={}
    var bgm_list={}
    var graph_file=null
    var mp_file=null
    var frame_file=null
    var voice_list_file=null
    var landmark_list_file=null
    for (var file_id=0; file_id<this.files.length; file_id++){
        var file=this.files[file_id]
        var path=file["webkitRelativePath"]
        var path_vec = path.split("/")
        if (path_vec.length<2){
            continue
        }
        var file_name=path_vec[path_vec.length-1]
        var folder_name=path_vec[path_vec.length-2]
        if (file_name.includes(".jpg")){
            if(folder_name=="res"){
                var reader = new FileReader()
                reader.onload =(function(){
                    var fileName = file_name
                    return function(e){
                        var loader = new THREE.TextureLoader();
                        loader.load(e.target.result, load_img_res_callback(fileName))
                    };
                })();
                reader.readAsDataURL(file);
            }else if(folder_name=="landmark"){
                lm_jpg_files[file_name]=file
            }else if(folder_name=="imgs"){
                file_list[parseInt(file_name.split(".jpg")[0])]=file
            }
        }
        if (file_name.includes("graph.json")){
            graph_file=file
        }
        if (file_name.includes("mps_seg.json")){
            mp_file=file
        }
        if (file_name.includes("frames.json")){
            frame_file=file
        }
        if (file_name.includes("voice.json")){
            voice_list_file=file
        }
        if (file_name.includes("landmark.json")){
            landmark_list_file=file
        }
        if (file_name.includes(".mp3")){
            if (folder_name=="bgm"){
                bgm_file=file
            }else if (folder_name=="voice"){
                mp3_list[file_name]=file
            }else if (folder_name=="landmark"){
                lm_mp3_files[file_name]=file
            }
        }
    }
    var frame_list=[]
    if (frame_file!=null){
        var fr = new FileReader()
        fr.onload = function(e) {
            var lines = e.target.result
            frame_list= JSON.parse(lines)
            if (graph_file!=null){
                var fr = new FileReader();
                fr.onload = function(e) {
                    var lines = e.target.result;
                    var obj = JSON.parse(lines)
                    graph["nodes"]=[]
                    for (var i=0; i<obj["node"].length; i++){
                        raw_node=obj["node"][i]
                        img_id=frame_list[raw_node["id"]][0]
                        if (img_id in file_list){
                            var node={}
                            node["id"]=i
                            node["c"]=[]
                            node["f"]=file_list[img_id]
                            node["posi"]=frame_list[raw_node["id"]][1]
                            node["dir"]=frame_list[raw_node["id"]][2]
                            graph["nodes"].push(node)
                        }else{
                            alert("img file miss: "+img_id);
                            break
                        }
                    }
                    for (var i=0; i<obj["conn"].length; i++){
                        var c=obj["conn"][i]
                        graph["nodes"][c["v1"]]["c"].push(graph["nodes"][c["v2"]])
                        graph["nodes"][c["v2"]]["c"].push(graph["nodes"][c["v1"]])
                        graph["edges"].push([graph["nodes"][c["v1"]],graph["nodes"][c["v2"]]])
                    }
                    cur_node=graph["nodes"][0]
                    next_node=graph["nodes"][0]
                    show_image(cur_node)
                    show_graph_3d()
                    play_thread(file_list)
                    update_scene_map()
                    update_voice_list()
                    update_landmarks()
                    update_bgms()
                    show_all_landmarks()
                };
                fr.readAsText(graph_file);
            }
        }
        fr.readAsText(frame_file)
    }
    if (mp_file!=null){
        show_pc(mp_file)
    }
    if (voice_list_file!=null){
        var fr = new FileReader();
        fr.onload = function(e) {
            var lines = e.target.result;
            voice_list = JSON.parse(lines)
        };
        fr.readAsText(voice_list_file);
    }
    if (landmark_list_file!=null){
        var fr = new FileReader();
        fr.onload = function(e) {
            var lines = e.target.result;
            landmarks = JSON.parse(lines)
        };
        fr.readAsText(landmark_list_file);
    }
}

function cal_dist(v1,v2){
    return Math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1])+(v1[2]-v2[2])*(v1[2]-v2[2]))
}

function update_voice_list(){
    var display_list=[]
    for(var i=0;i<voice_list.length; i++){
        var range=voice_list[i][3]
        var tmp_posi = graph["nodes"][voice_list[i][0]]["posi"]
        var dist = cal_dist(cur_node["posi"],tmp_posi)
        if (dist<range){
            display_list.push(i)
        }
    }
    var str=""
    for(var i=0;i<display_list.length;i++){
        var voice_id=display_list[i]
        str=str+'<input type="button" onclick="window.del_voice('+voice_id.toString()+')" value="D"/><a onclick="window.play_voice('+voice_id.toString()+')">'+voice_list[voice_id][2]+'</a></br>'
    }
    document.getElementById("voice_list").innerHTML=str
}

function del_voice(del_ind){
    var new_voice_list=[]
    for (var i=0; i<voice_list.length; i++){
        if (i!=del_ind){
            new_voice_list.push(voice_list[i])
        }
    }
    voice_list=new_voice_list
    update_voice_list()
}

function play_voice(ind){
    var voice_name=voice_list[ind][1]
    if (voice_name ==cur_voice["name"]){
        if (cur_voice["dom"].paused){
            cur_voice["dom"].play()
        }else{
            cur_voice["dom"].pause()
        }
        return
    }
    if (cur_voice["name"]=="" || voice_name !=cur_voice["name"]){
        if (cur_voice["name"]!=""){
            cur_voice["dom"].pause()
        }
        var mp3_file=mp3_list[voice_name]
        var reader = new FileReader();
        reader.onload = function (e) {
            var soundFile = document.createElement("audio")
            soundFile.preload = "auto"
            var src = document.createElement("source")
            src.src = e.target.result
            soundFile.appendChild(src)
            soundFile.load()
            soundFile.play()
            cur_voice={"name":voice_name, "dom":soundFile}
        };
        reader.readAsDataURL(mp3_file);
    }
}

function download_voice_list(){
    var tmp_str=JSON.stringify(voice_list)
    var a = document.createElement("a")
    var file = new Blob([tmp_str], {type: "json"});
    a.href = URL.createObjectURL(file);
    a.download = "voice.json";
    a.style.display = "none"
    a.click()
}

function send_voice(){
    var mp3_file_dom = document.getElementById("load_mp3");
    var title=document.getElementById("voice_title").value
    var range=parseInt(document.getElementById("v_range").value)
    if (title==""){
        return
    }
    if (cur_node==null){
        return
    }
    if ('files' in mp3_file_dom) {
        if (mp3_file_dom.files.length != 0) {
            var mp3_file=mp3_file_dom.files[0]
            if (mp3_file["name"].includes(".mp3")){
                var new_voice=[cur_node["id"],mp3_file["name"],title,range]
                voice_list.push(new_voice)
                mp3_list[mp3_file["name"]]=mp3_file
            }
        }
    }
    update_voice_list()
}

function check_lm_img(b_true){
    var val=document.getElementById("b_lm_img").value
    if (b_true==null){
        b_true=val
    }
    var div_tmp = document.getElementById("lm_img")
    console.log(b_true)
    if (b_true){
        div_tmp.style.display = "none"
    }else{
        div_tmp.style.display = "block"
    }
}

function check_bgm(b_true){
    console.log(b_true)
    if (b_true){
        soundBGM.play()
    }else{
        soundBGM.pause()
    }
}

$(document).ready(function(){
    window.del_voice = del_voice
    window.play_voice = play_voice
    window.download_voice_list=download_voice_list
    init_pano()
    init_scene()
    animate()
    document.getElementById("send_v").addEventListener('click', send_voice);
    var inputNode = document.getElementById("load_folder");
    inputNode.addEventListener('change', process_folder, false)
    document.addEventListener('keydown', function(event) {
        if (event.defaultPrevented) {
            return;
        }
        if(event.keyCode == 87) {
            up_key=true
        }else if(event.keyCode == 83) {
            back_key=true
        }else if(event.keyCode == 68) {
            right_key=true
        }else if(event.keyCode == 65) {
            left_key=true
        }
    });
    document.addEventListener('keyup', function(event) {
        if (event.defaultPrevented) {
            return;
        }
        if(event.keyCode == 87) {
            up_key=false
        }else if(event.keyCode == 83) {
            back_key=false
        }else if(event.keyCode == 68) {
            right_key=false
        }else if(event.keyCode == 65) {
            left_key=false
        }
    });
})

