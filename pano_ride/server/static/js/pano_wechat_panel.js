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
var cur_node=null
var next_node=null
var traj_threejs=[]
var me=null
var mp3_list={}
var cur_voice={"name":"","dom":null}
var moving_speed=1
var bgm_dom=null
var soundBGM
var video_dom=null
var steer_dom_query=null
var steer_posi=null
var steer_size=null
var debug_info_query=null
var map_dom_h=null
var steer_dots=[]
var steer_status=1000
var video_can_play=false
var steer_qanel_query=null
var me=null
var dir_marker_threejs=null

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
        rot_steer_dot()
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
//    var img_dir=cur_node["dir"]
    img_dir=0
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
    container.style.height=(winHeight*0.4).toString()+"px"
    map_dom_h=winHeight*0.4
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
    orbitControls = new THREE.OrbitControls( camera_map, renderer_map.domElement )
    orbitControls.addEventListener( 'change', render_map );
    orbitControls.maxPolarAngle=2.35619445
    orbitControls.touches = { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_ROTATE }
}

function render_map(){
    renderer_map.render(scene_map, camera_map)
}

function anime(){
    requestAnimationFrame(anime)
    orbitControls.update()
    update_pano()
}

function cal_dist(v1,v2){
    return Math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1])+(v1[2]-v2[2])*(v1[2]-v2[2]))
}

function check_bgm(b_true){
    if (b_true){
        soundBGM.play()
    }else{
        soundBGM.pause()
    }
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
            
            after_load_graph(data)
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

function clear_steer_dots(){
    for(var i=0; i<steer_dots.length; i++){
        steer_dots[i][0].remove()
    }
    steer_dots=[]
}

function add_steer_dot(angle){
    var tmp_dot_query=$('<div class="dot"></div>')
    tmp_dot_query.appendTo(steer_qanel_query)
    var angle_rad= angle
    steer_dots.push([tmp_dot_query,angle_rad])
    angle_rad=angle_rad-degree_2_rad(lon)
    var dot_r=steer_size
    var dot_y= -Math.cos(angle_rad)*dot_r+steer_posi[0]-15
    var dot_x= Math.sin(angle_rad)*dot_r+steer_posi[1]-15
    tmp_dot_query.css({top: dot_y, left: dot_x, position:'absolute'})
}

function rot_steer_dot(){
    for(var i=0; i<steer_dots.length; i++){
        new_angle=steer_dots[i][1]-degree_2_rad(lon)
        var dot_r=steer_size
        var dot_y= -Math.cos(new_angle)*dot_r+steer_posi[0]-15
        var dot_x= Math.sin(new_angle)*dot_r+steer_posi[1]-15
        steer_dots[i][0].css({top: dot_y, left: dot_x, position:'absolute'})
    }
}

function show_image(node){
    var time_stamp=node["id"]*0.04
    video_dom.currentTime=time_stamp
    cur_node=node
}

function update_steer(){
    if (cur_node==null){
        return
    }
    clear_steer_dots()
    for (var i=0; i<cur_node["c"].length; i++){
        var angle = get_angle_from_mat(cur_node["posi"], cur_node["c"][i]["posi"])
        add_steer_dot(angle)
    }
}

function add_v(v1, v2){
    return [v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2]]
}

function update_dir_marker(){
    if (cur_node==null){
        return
    }
    var img_posi= cur_node["posi"]
    if (dir_marker_threejs!=null){
        scene_map.remove(dir_marker_threejs)
        dir_marker_threejs=null
    }
    var rad_angle=degree_2_rad(lon)
    var dir=[Math.sin(rad_angle)*0.5, 0,Math.cos(rad_angle)*0.5]
    var dir_vec=[img_posi,add_v(dir, img_posi)]
    dir_marker_threejs=add_threejs_segment([dir_vec],"#ff0000", false)
}

function update_scene_map(){
    if (graph==null){
        return
    }
    var all_nodes=[]
    var all_edges=[]
    for (var i=0; i<graph["nodes"].length; i++){
        all_nodes.push(graph["nodes"][i]["posi"])
    }
    for (var i=0; i<graph["edges"].length; i++){
        var tmp_edge = graph["edges"][i]
        var edge=[]
        edge.push(tmp_edge[0]["posi"])
        edge.push(tmp_edge[1]["posi"])
        all_edges.push(edge)
    }
    var color="#ffffff"
    node_threejs.push(add_threejs_pts(all_nodes, color, 5))
    edge_threejs.push(add_threejs_segment(all_edges, color))
    if (cur_node!=null){
        var p_tmp=cur_node["posi"]
        me.position.set(p_tmp[0], p_tmp[2], -p_tmp[1]+0.01)
        update_dir_marker()
        render_map()
    }
    
}

function get_angle_2d(ori, tar){
    var dir=[tar[0]-ori[0], tar[1]-ori[1]]
    var angle = Math.atan2( dir[1], dir[0] )
    return angle
}

function onSteerTouchDown(evt){
    evt.preventDefault();
    var touches = evt.originalEvent.touches;
    if (touches.length<=0){
        return
    }
    var touch_posi=[touches[0].pageX, touches[0].pageY]
    steer_status=get_angle_2d(steer_posi_screen_center, touch_posi)
    debug_info_query.html("down")
    console.log("down")
}

function onSteerTouchMove(evt){
    evt.preventDefault();
    var touches = evt.originalEvent.touches;
    if (touches.length<=0){
        return
    }
    var touch_posi=[touches[0].pageX, touches[0].pageY]
//    debug_info_query.html("move")
//    console.log("move")
//    steer_status=get_angle_2d(steer_posi_screen_center, touch_posi)
}
function onSteerTouchUp(evt){
    evt.preventDefault()
//    debug_info_query.html("up")
//    console.log("up")
//    steer_status=1000
}
function onSteerTouchOut(evt){
    evt.preventDefault()
//    debug_info_query.html("out")
//    console.log("out")
//    steer_status=1000
}

function get_angle_from_mat(ori, tar){
    var dir=[tar[2]-ori[2], tar[0]-ori[0]]
    var angle = Math.atan2( dir[1], dir[0] )
    return angle
}

function sleep(ms) {
   return new Promise(resolve => setTimeout(resolve, ms));
}

async function play_thread(){
   while (true){
       if (cur_node["id"]==null || next_node["id"]==null){
           await sleep(100)
       }
       if (cur_node["id"]!=next_node["id"]){
           show_image(next_node)
           update_scene_map()
           update_steer()
           var tmp_count=0
           while(true){
               tmp_count=tmp_count+1
               if (video_dom.seeking==false){
                   break
               }
               await sleep(100)
           }
       }
       var angle=rad_2_degree(steer_status)
       if (angle>=1000){
           await sleep(100)
       }else{
           steer_status=1000
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
               cam_angle_w=rad_2_degree(cam_angle_w)
               
               var angle_diff = angle_world-cam_angle_w-270
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
               wait_time=moving_speed*200
               //await sleep(wait_time)
           }else{
               await sleep(100)
           }
       }
   }
}

function go(){
    var start_btn_dom=$("#start_btn");
    start_btn_dom.css({"display":"none"})
    if (video_can_play==false){
        alert("video_can_play==false");
        video_dom.play()
    }else{
        alert("video_can_play==true");
    }
}

function degree_2_rad(degree){
    return degree*Math.PI/180
}

function rad_2_degree(rad){
    return rad/Math.PI*180
}

function after_load_graph(raw_graphs){
    tmp_pts=[]
    tmp_pts.push([0,0,0])
    me = add_threejs_pts(tmp_pts,"#55eeff",10)
    graph["nodes"]=[]
    graph["edges"]=[]
    for (var i=0; i<raw_graphs["node"].length; i++){
        raw_node=raw_graphs["node"][i]
        var node={}
        node["id"]=i
        node["c"]=[]
        node["posi"]=raw_node[0]
        node["dir"]=rad_2_degree(raw_node[1])
        graph["nodes"].push(node)
    }
    for (var i=0; i<raw_graphs["edge"].length; i++){
        var c=raw_graphs["edge"][i]
        graph["nodes"][c[0]]["c"].push(graph["nodes"][c[1]])
        graph["nodes"][c[1]]["c"].push(graph["nodes"][c[0]])
        graph["edges"].push([graph["nodes"][c[1]],graph["nodes"][c[0]]])
    }
    cur_node=graph["nodes"][0]
    next_node=graph["nodes"][0]
    update_scene_map()
    update_steer()
    render_map()
}

function GetScreenCordinates(obj) {
    var p = {};
    p.x = obj.offsetLeft;
    p.y = obj.offsetTop;
    while (obj.offsetParent) {
        p.x = p.x + obj.offsetParent.offsetLeft;
        p.y = p.y + obj.offsetParent.offsetTop;
        if (obj == document.getElementsByTagName("body")[0]) {
            break;
        }
        else {
            obj = obj.offsetParent;
        }
    }
    return p;
}

$(document).ready(function(){
    $.ajaxSetup({
        async: true
    })
    var url_paremeters_dic=GetRequest()
    var map_name = url_paremeters_dic['n']
    video_dom=document.getElementById('video_pano')
    video_dom.src="https://ride-v.oss-cn-beijing.aliyuncs.com/pano_maps/"+map_name+"/chamo.mp4"
    video_dom.oncanplay = function() {
        video_dom.pause()
        video_can_play=true
    };
//    video_dom.ontimeupdate = function() {
//        //alert("timeupdate");
//    };
//    video_dom.onloadstart = function() {
//        alert("loadstart");
//    };
//    video_dom.ondurationchange = function() {
//        alert("durationchange");
//    };
//    video_dom.onloadedmetadata = function() {
//        alert("loadedmetadata");
//    };
//    video_dom.onloadeddata = function() {
//        alert("loadeddata");
//    };
//    video_dom.onprogress = function() {
//        //alert("progress");
//    };
//    video_dom.oncanplay = function() {
//        alert("canplay");
//    };
//    video_dom.oncanplaythrough = function() {
//        alert("canplaythrough");
//    };
//    video_dom.onseeked = function() {
//        alert("seeked");
//    };
//    video_dom.onseeking = function() {
//        alert("seeking");
//    };
    video_dom.onplay = function() {
        alert("play");
    };
    
    
    steer_qanel_query=$("#wechat_panel")
    var winHeight = window.innerHeight;
    var winWidth = window.innerWidth;
    steer_qanel_query.height(winHeight*0.1)
    steer_qanel_query.width(winHeight*0.1)
    steer_dom_query=$("#steer")
    steer_dom_query.height("100%")
    steer_dom_query.width("100%")
    var steer_panel_size=steer_qanel_query.width()/2+40
    steer_size=steer_dom_query.width()/2
    steer_posi=[steer_panel_size,steer_panel_size]
    var tmp_obj=document.getElementById("wechat_panel")
    var tmp_posi=GetScreenCordinates(tmp_obj)
    steer_posi_screen_center=[tmp_posi.x+steer_posi[0],tmp_posi.y+steer_posi[1]]
    steer_qanel_query.on('touchstart', onSteerTouchDown);
    steer_qanel_query.on( 'touchmove', onSteerTouchMove);
    steer_qanel_query.on( 'touchend', onSteerTouchUp);
    steer_qanel_query.on( 'touchcancel', onSteerTouchOut);
    
    debug_info_query=$("#wechat_debug")
    init_scene()
    init_pano()
    fetch_json("graph.json", map_name)
    play_thread()
    anime()
})

