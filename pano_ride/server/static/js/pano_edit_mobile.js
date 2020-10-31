var camera_pano, scene_pano, renderer_pano
var camera_map, scene_map, renderer_map
var orbitControls
var graph={}
var node_threejs=[]
var edge_threejs=[]
var isUserInteracting = false,
    lon = 0, lat = 0,
    distance = 1.1,
    onPointerDownPointerX = 0,
    onPointerDownPointerY = 0,
    onPointerDownLon = 0,
    onPointerDownLat = 0;
var cur_frameid=0
var traj_threejs=[]
var frame_objs=[]
var video_dom=null
var map_dom_h=null
var video_can_play=false
var me=null
var dir_marker_threejs=null
var segments=null
var frames=null
var frame_2_segment_table=[]
var map_name=""
var last_touch_pt=[]
var start_seg_id=-1
var end_seg_id=-1
var chosed_seg_obj=null
var segs_objs=[]

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

function add_threejs_lines(pts,color, line_w){
    var points = []
    for (var i=0; i<pts.length; i++){
        points.push( new THREE.Vector3(pts[i][0], pts[i][2], -pts[i][1]))
    }
    var trajGeometry = new THREE.Geometry()
    trajGeometry.setFromPoints( points )
    var trajLine = new THREE.Line( trajGeometry, new THREE.LineBasicMaterial({
        color: color, linewidth: line_w, opacity: 1
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

function init_pano() {
    var container,container_query, mesh;
    container = document.getElementById('wechat_pano');
    var winHeight = window.innerHeight;
    var winWidth = window.innerWidth;
    container.style.height=(winHeight-map_dom_h).toString()+"px"
    container_query=$("#wechat_pano");
    var divWidth = container_query.width()
    var divHeight = container_query.height()
    var start_btn_dom=$("#start_btn");
    var btn_h=container_query.position().top+divHeight/2
    start_btn_dom.css({"top": btn_h,"height":divHeight*0.1,"display":"block"})
    var rate=divWidth/divHeight
    camera_pano = new THREE.PerspectiveCamera( 75, rate, 1, 1100 );
    camera_pano.target = new THREE.Vector3( 0, 0, 0 );
    scene_pano = new THREE.Scene();
    var geometry = new THREE.SphereBufferGeometry( 1000, 60, 40 );
    geometry.scale( -1, 1, 1 );
    var video = document.getElementById('video_pano')
    var texture = new THREE.VideoTexture( video );
    var material = new THREE.MeshBasicMaterial( { map: texture } );
    mesh = new THREE.Mesh( geometry, material );
    scene_pano.add( mesh );
    renderer_pano = new THREE.WebGLRenderer();
    renderer_pano.setPixelRatio( window.devicePixelRatio );
    renderer_pano.setSize( divWidth, divHeight);
    container.appendChild( renderer_pano.domElement );
    container_query.on( 'touchstart', onPanoTouchDown);
    container_query.on( 'touchmove', onPanoTouchMove);
    container_query.on( 'touchend', onPanoTouchUp);
    container_query.on( 'touchcancel', onPanoTouchOut);
}


function onPanoTouchOut(){
    isUserInteracting=false
}

function onPanoTouchDown( evt ) {
    evt.preventDefault();
    isUserInteracting = true;
    var touches = evt.originalEvent.touches;
    if (touches.length<=0){
        return
    }
    onPointerDownPointerX = touches[0].pageX;
    onPointerDownPointerY = touches[0].pageY;
    onPointerDownLon = lon;
    onPointerDownLat = lat;
}

function onPanoTouchMove( evt ) {
    if ( isUserInteracting === true ) {
        var touches = evt.originalEvent.touches;
        if (touches.length<=0){
            return
        }
        lon = ( onPointerDownPointerX - touches[0].pageX ) * 0.1 + onPointerDownLon;
        lat = ( touches[0].pageY - onPointerDownPointerY ) * -0.1 + onPointerDownLat;
        if (lat>20){
           onPointerDownPointerY=onPointerDownPointerY-(lat-20)
           lat=20
        }
        if (lat<-27){
           onPointerDownPointerY=onPointerDownPointerY+(-27-lat)
           lat=-27
        }
        update_dir_marker()
        render_map()
    }
}
function onPanoTouchUp() {
    isUserInteracting = false
}

function update_pano() {
    if (video_can_play==false){
        return
    }
//    if (cur_node==null){
//        return
//    }
    phi = THREE.Math.degToRad( 90 - lat );
    var img_dir=frames[cur_frameid][2]
    //img_dir=0
    theta = THREE.Math.degToRad( lon-img_dir);
    camera_pano.position.x = distance * Math.sin( phi ) * Math.cos( theta );
    camera_pano.position.y = distance * Math.cos( phi );
    camera_pano.position.z = distance * Math.sin( phi ) * Math.sin( theta );
    camera_pano.lookAt( camera_pano.target );
    renderer_pano.render( scene_pano, camera_pano );
}

function init_scene() {
    scene_map = new THREE.Scene()
    var container,container_query;
    container = document.getElementById('wechat_map');
    var winHeight = window.innerHeight;
    var winWidth = window.innerWidth;
    container.style.height=(winHeight*0.6).toString()+"px"
    map_dom_h=winHeight*0.6
    container_query=$("#wechat_map");
    var divWidth = container_query.width()
    var divHeight = container_query.height()
    camera_map = new THREE.OrthographicCamera( divWidth/-50, divWidth/50, divHeight/50, divHeight/-50, 0.1, 10000 )
    camera_map.position.set(0, 0, 500);
    camera_map.up.x = 0;
    camera_map.up.y = 0;
    camera_map.up.z = 1;
    camera_map.lookAt(0, 0, 0);
    camera_map.aspect = divWidth/divHeight;
    camera_map.updateProjectionMatrix();
    renderer_map = new THREE.WebGLRenderer();
    renderer_map.setClearColor(new THREE.Color(0x000000));
    container.appendChild(renderer_map.domElement);
    renderer_map.setSize( divWidth, divHeight );
    renderer_map.domElement.addEventListener("touchstart", on_touchstart, true);
    renderer_map.domElement.addEventListener("touchmove", on_touchmove, true);
    renderer_map.domElement.addEventListener("touchend", onclick, true);
    orbitControls = new THREE.OrbitControls( camera_map, renderer_map.domElement )
    orbitControls.addEventListener( 'change', render_map );
    orbitControls.maxPolarAngle=2.35619445
    orbitControls.touches = { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_ROTATE }
}

function on_touchstart(event){
    last_touch_pt=event.touches[0]
}

function on_touchmove(event){
    last_touch_pt=null
}

function onclick(){
    if (last_touch_pt==null){
        return
    }
    var mouse = new THREE.Vector2()
    event.preventDefault();
    container_query=$("#wechat_map");
    var divWidth = container_query.width()
    var divHeight = container_query.height()
    mouse.x = ( last_touch_pt.pageX / divWidth) * 2 - 1;
    mouse.y = - ( last_touch_pt.pageY / divHeight ) * 2 + 1;
    var marker_raycaster = new THREE.Raycaster();
    if (frame_objs.length>0){
        marker_raycaster.setFromCamera(mouse, camera_map)
        var intersects = marker_raycaster.intersectObjects(frame_objs, true)
        if (intersects.length > 0) {
            var min_dist=null
            var min_dist_id=-1
            for(var i=0; i<intersects.length; i++){
                if (min_dist==null || min_dist>intersects[i].distanceToRay){
                    min_dist=intersects[i].distanceToRay
                    min_dist_id=intersects[i].index
                }
            }
            cur_frameid=min_dist_id
            request_frame()
            update_me()
        }
    }
}

function update_me(){
    posi=frames[cur_frameid][1]
    me.position.set(posi[0], posi[2], -posi[1])
    update_dir_marker()
    render_map()
}

function render_map(){
    renderer_map.render(scene_map, camera_map)
}

function anime(){
    requestAnimationFrame(anime)
    orbitControls.update()
}

function cal_dist(v1,v2){
    return Math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1])+(v1[2]-v2[2])*(v1[2]-v2[2]))
}

function fetch_json(file_name, map_name){
    var out_data=null
    var oss_root="https://ride-v.oss-cn-beijing.aliyuncs.com/pano_maps"
    var url = oss_root+"/"+map_name+"/"+file_name;
    $.ajax({
        url: url,
        dataType: 'json',
        async: false,
        success: function(data) {
            if (file_name=="segments.json"){
                segments=data
            }
            if (file_name=="frames.json"){
                frames=data
            }
            after_load_data()
        },
        error: function(XMLHttpRequest, textStatus, errorThrown) {
        }
    });
}

function GetRequest() {
   var url = location.search;
   var theRequest = new Object();
   if (url.indexOf("?") != -1) {
      var str = url.substr(1);
      strs = str.split("?");
      for (var i = 0; i < strs.length; i++) {
          theRequest[strs[i].split("=")[0]] = decodeURIComponent(strs[i].split("=")[1]);
       }
   }
   return theRequest;
}

function add_v(v1, v2){
    return [v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2]]
}

function update_dir_marker(){
    var img_posi= frames[cur_frameid][1]
    if (dir_marker_threejs!=null){
        scene_map.remove(dir_marker_threejs)
        dir_marker_threejs=null
    }
    var rad_angle=degree_2_rad(lon)
    var dir=[Math.sin(rad_angle)*0.5, 0,Math.cos(rad_angle)*0.5]
    var dir_vec=[img_posi,add_v(dir, img_posi)]
    dir_marker_threejs=add_threejs_segment([dir_vec],"#ff6688", false)
}

function get_angle_2d(ori, tar){
    var dir=[tar[0]-ori[0], tar[1]-ori[1]]
    var angle = Math.atan2( dir[1], dir[0] )
    return angle
}

function get_angle_from_mat(ori, tar){
    var dir=[tar[2]-ori[2], tar[0]-ori[0]]
    var angle = Math.atan2( dir[1], dir[0] )
    return angle
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function request_frame(){
    if (video_dom.seeking){
        return
    }
    video_dom.currentTime=cur_frameid*0.04
}

function go(){
    var start_btn_dom=$("#start_btn");
    start_btn_dom.css({"display":"none"})
    if (video_can_play==false){
        video_dom.play()
    }
}

function degree_2_rad(degree){
    return degree*Math.PI/180
}

function rad_2_degree(rad){
    return rad/Math.PI*180
}

function update_chosed_seg(){
    if (chosed_seg_obj!=null){
        scene_map.remove(chosed_seg_obj)
    }
    if (start_seg_id>=0 && end_seg_id>=0){
        var start_id=start_seg_id
        var end_id=end_seg_id+1
        if (start_id>end_seg_id){
            start_id=end_seg_id
            end_id=start_seg_id+1
        }
        var posis=[]
        for (var i=start_id; i<end_id; i++){
            posis.push(frames[i][1])
        }
        if (chosed_seg_obj!=null){
            scene_map.remove(chosed_seg_obj)
        }
        chosed_seg_obj = add_threejs_lines(posis,"#ff66ff",5)
    }
    render_map()
}

function add_seg(){
    if (start_seg_id==-1 || end_seg_id==-1){
        return
    }
    related_segs=[]
    var start_id=start_seg_id
    var end_id=end_seg_id+1
    if (start_id>end_id){
        start_id=end_seg_id
        end_id=start_seg_id+1
    }
    for (var i=start_id; i<end_id; i++){
        if (frame_2_segment_table[i]!=-1){
            if (!related_segs.includes(frame_2_segment_table[i])){
                related_segs.push(frame_2_segment_table[i])
            }
        }
    }
    remain_segs=[]
    for (var i=0; i<segments.length; i++){
        if (!related_segs.includes(i)){
            remain_segs.push(segments[i])
        }
    }
    segments=remain_segs
    segments.push([-1,start_id,end_id])
    update_seg()
    start_seg_id=-1
    end_seg_id=-1
    update_chosed_seg()
}

function del_seg(){
    var tmp_segid = frame_2_segment_table[cur_frameid]
    if (frame_2_segment_table[tmp_segid]==-1){
        return
    }
    remain_segs=[]
    for (var i=0; i<segments.length; i++){
        if (tmp_segid!=i){
            remain_segs.push(segments[i])
        }
    }
    segments=remain_segs
    update_seg()
}
function output(){
    $.ajax({
        type: 'GET',
        url: '../../process_seg_modify',
        data: {map_name:map_name, segments:JSON.stringify(segments)},
        dataType: 'json',
        success: function(data) {
            if (data[0]=="ok"){
            }
        },
    });
}

function update_frame_table(){
    frame_2_segment_table=[]
    for (var i=0; i<frames.length; i++){
        var find_seg=-1
        for (var j=0; j<segments.length; j++){
            if (segments[j][1]<=i && segments[j][2]>=i){
                find_seg=j
            }
        }
        frame_2_segment_table.push(find_seg)
    }
}

function update_seg(){
    if (segments ==null || frames==null){
        return
    }
    for (var i=0; i<segs_objs.length; i++){
        scene_map.remove(segs_objs[i])
    }
    for (var i=0; i<segments.length; i++){
        var posis=[]
        for (var j=segments[i][1]; j<segments[i][2]+1; j++){
            posis.push(frames[j][1])
        }
        segs_objs.push(add_threejs_lines(posis,"#ffaa88",2))
    }
    render_map()
    update_frame_table()
}

function update_frames(){
    if (frames==null){
        return
    }
    var posis=[]
    for (var i=0; i<frames.length; i++){
        posis.push(frames[i][1])
    }
    frame_objs.push(add_threejs_pts(posis,"#ffee66",5))
}

function after_load_data(){
    if (segments ==null || frames==null){
        return
    }
    tmp_pts=[]
    tmp_pts.push([0,0,0])
    me = add_threejs_pts(tmp_pts,"#55eeff",10)
    update_frames()
    update_seg()
    render_map()
    update_me()
    update_pano()
}

function set_start(){
    start_seg_id=cur_frameid
    update_chosed_seg()
}
function set_end(){
    end_seg_id=cur_frameid
    update_chosed_seg()
}

$(document).ready(function(){
    $.ajaxSetup({
        async: true
    })
    var url_paremeters_dic=GetRequest()
    map_name = url_paremeters_dic['n']
    btns_query=$(".mobile_button")
    console.log(btns_query)
    var winHeight = window.innerHeight;
    var winWidth = window.innerWidth;
    for (var i=0; i<btns_query.length; i++){
        btns_query[i].style.width=(winWidth*0.15).toString()+"px"
        btns_query[i].style.height=(winWidth*0.15*0.25).toString()+"px"
    }
    video_dom=document.getElementById('video_pano')
    video_dom.src="https://ride-v.oss-cn-beijing.aliyuncs.com/pano_maps/"+map_name+"/chamo.mp4"
    video_dom.oncanplay = function() {
        video_dom.pause()
        video_can_play=true
        update_pano()
    };
    video_dom.ontimeupdate = function() {
    };
    video_dom.onseeked = function() {
        update_pano()
    };
    video_dom.onseeking = function() {
        //debug_info_query.html("seeking")
    };
    debug_info_query=$("#wechat_debug")
    init_scene()
    init_pano()
    fetch_json("segments.json", map_name)
    fetch_json("frames.json", map_name)
    anime()
})

