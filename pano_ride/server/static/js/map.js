var trajs=[]
var texture_cache=[]
var leaflet_objs=[]
var cur_traj_data=[]
var cur_sel_img=""
var cur_path=[]
var cur_img_marker=null
var s_img=""
var d_img=""
var mymap
var camera, scene, renderer
var isUserInteracting = false,
    lon = 0, lat = 0,
    phi = 0, theta = 0,
    distance = 1.1,
    onPointerDownPointerX = 0,
    onPointerDownPointerY = 0,
    onPointerDownLon = 0,
    onPointerDownLat = 0;
var fix_lon_offset=114
var fix_lon_offset_mark=-65
var img_root="https://ride-v.oss-cn-beijing.aliyuncs.com/maps/shangrila/raw_data/"
var pano_material
const loader = new THREE.TextureLoader();
var caching=false
var stop_caching_flag=false
var stop_playing_flag=false
var is_pasue=true
var cur_play_pos=-1
var cur_path_obj=null
var is_traj_caching=false

function show_map(){
    var map_root = document.getElementById("panel_1_2")
    var div_map = document.createElement("div")
    div_map.id="map"
    map_root.appendChild(div_map)
    mymap = L.map(div_map)
    var baseLayers1 = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 25,
        maxNativeZoom: 19
    })
    var baseLayers3 = L.tileLayer('https://{s}.tile.thunderforest.com/transport/{z}/{x}/{y}.png?apikey={apikey}', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        apikey: '<your apikey>',
        minZoom: 0,
        maxZoom: 25,
        maxNativeZoom: 19,
        ext: 'png'
    });
    var baseLayers4 = L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        subdomains: 'abcd',
        minZoom: 0,
        maxZoom: 25,
        maxNativeZoom: 19,
        ext: 'png'
    });
    var basemapControl = {
        "Light Map": baseLayers4,
        "Detail Map": baseLayers1,
        "Hide Map": baseLayers3
    }
    mymap.addLayer(baseLayers4);
    L.control.layers( basemapControl).addTo( mymap )
    L.control.scale({imperial: false}).addTo(mymap);
    L.control.polylineMeasure({}).addTo(mymap);
    mymap.setView([51.505, -0.09], 18);
}
function fetch_traj_list(lev, center){
    $.ajax({
        type: 'GET',
        url: '../../fetch_traj_list',
        dataType: 'json',
        success: function(data) {
            console.log(data)
            trajs=data
        },
    });
}
function fetch_traj_data(traj_id){
    $.ajax({
        type: 'GET',
        url: '../../fetch_traj_data',
        data: {traj_id:traj_id},
        dataType: 'json',
        success: function(data) {
            cur_traj_data=data
            show_traj()
        },
    });
}
function show_traj(){
    for (var i=0; i<cur_traj_data.length; i=i+1) {
        var slope_pts_data=[cur_traj_data[i][1], cur_traj_data[i][0]]
        traj_pt_circle = L.circle(slope_pts_data,{color:'black',fillColor:'black',radius:1.5,fillOpacity:1,zIndexOffset:1000}).addTo(mymap)
        .on('click', function(e) {
                show_img(e["target"]["img_id"])
                cur_sel_img=e["target"]["img_id"]
                latlon=e["target"].getLatLng()
                move_cur_img_marker([latlon["lat"],latlon["lng"]])
            });
        traj_pt_circle["img_id"]=cur_traj_data[i][2]
        leaflet_objs[cur_traj_data[i][2]]=traj_pt_circle
    }
}
function show_img_gpss(){
    testCluster = L.markerClusterGroup({
        showCoverageOnHover: false,
        chunkedLoading: true,
        singleMarkerMode: true,
        spiderfyOnMaxZoom: false,
        removeOutsideVisibleBounds: true,
        disableClusteringAtZoom: 18,
        maxClusterRadius: 30
    });
    testCluster.on('clustermouseover', function (a) {
    });
    last_pt=[]
    for (var i=0; i<trajs.length; i++) {
        info_tmp=trajs[i]
        last_pt=[info_tmp[1], info_tmp[0]]
//        console.log(last_pt)
        var traj_marker = L.marker([info_tmp[1], info_tmp[0]]).addTo(testCluster)
        traj_marker.on('click',
            L.bind(function(id) {
                fetch_traj_data(id)
                infor_str="Trajectory Name: "+id+"</br>"
                $('#traj_info').html(infor_str)
            },
        null, info_tmp[2]));
    }
    mymap.addLayer(testCluster);
    mymap.flyTo(last_pt, 18, {
        animate: false,
    })
}
function set_traj_pt_cached(image_id){
    if (image_id in leaflet_objs){
        leaflet_objs[image_id].setStyle({color:'red',fillColor:'red'})
    }
}
function callback(img_id, show_img, b_full_cache) {
    return (texture) => {
        set_traj_pt_cached(img_id, b_full_cache)
        texture_cache[img_id]=texture
        if (show_img){
            pano_material.map=texture
            pano_material.map.needsUpdate=true
        }
        caching=false
    }
}
function move_cur_img_marker(posi){
    if (cur_img_marker==null){
        cur_img_marker=L.circle(posi,{color:'black',radius:3.0,fillOpacity:0,interactive:false}).addTo(mymap)
    }
    var newLatLng = new L.LatLng(posi[0], posi[1]);
    cur_img_marker.setLatLng(newLatLng);
}
function show_img(image_id){
    if (image_id in texture_cache){
        pano_material.map=texture_cache[image_id]
        pano_material.map.needsUpdate=true
    }else{
        if (caching==true){
            return false
        }
        name_vec=image_id.split("-")
        real_img_name=name_vec[0]+"-"+name_vec[1]+"-"+name_vec[2]+"/imgs/"+name_vec[3]+".jpg"
        src=img_root+real_img_name
        caching=true
        loader.load(src, callback(image_id, true));
    }
    return true
}
function init_pano() {
    var container,container_query, mesh;
    container = document.getElementById( 'panel_1_1' );
    container_query=$("#panel_1_1");
    var w = container_query.width();
    var h = container_query.height();
    var rate=w/h
    camera = new THREE.PerspectiveCamera( 50, rate, 1, 1100 );
    camera.target = new THREE.Vector3( 0, 0, 0 );
    scene = new THREE.Scene();
    var geometry = new THREE.SphereBufferGeometry( 1000, 60, 40 );
    // invert the geometry on the x-axis so that all of the faces point inward
    geometry.scale( -1, 1, 1 );
    var texture =  new THREE.TextureLoader().load('#');
    pano_material = new THREE.MeshBasicMaterial( { map: texture } );
    mesh = new THREE.Mesh( geometry, pano_material );
    scene.add( mesh );
    renderer = new THREE.WebGLRenderer();
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( w, h);
    container.appendChild( renderer.domElement );
    container_query.on( 'mousedown', onDocumentMouseDown);
    container_query.on( 'mousemove', onDocumentMouseMove);
    container_query.on( 'mouseup', onDocumentMouseUp);
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
    lat = Math.max( - 27, Math.min( 9, lat ) );
    phi = THREE.Math.degToRad( 90 - lat );
    theta = THREE.Math.degToRad( lon+fix_lon_offset );
    camera.position.x = distance * Math.sin( phi ) * Math.cos( theta );
    camera.position.y = distance * Math.cos( phi );
    camera.position.z = distance * Math.sin( phi ) * Math.sin( theta );
    camera.lookAt( camera.target );
    renderer.render( scene, camera );
}
function set_start(){
    if (cur_sel_img!=""){
//        leaflet_objs[cur_sel_img].setStyle({color:'blue',fillColor:'blue'})
        s_img=cur_sel_img
    }
}
function set_dest(){
    if (cur_sel_img!=""){
//        leaflet_objs[cur_sel_img].setStyle({color:'blue',fillColor:'blue'})
        d_img=cur_sel_img
    }
}
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}
async function cache_img(){
    is_traj_caching=true
    for (var i =0; i<cur_path.length; i++){
        image_id = cur_path[i][2]
        if (image_id in texture_cache){
            continue
        }
        while(true){
            if (caching==true){
                await sleep(100)
            }else{
                break
            }
        }
        if (stop_caching_flag==true){
            stop_caching_flag=false
            break
        }
        name_vec=image_id.split("-")
        real_img_name=name_vec[0]+"-"+name_vec[1]+"-"+name_vec[2]+"/imgs/"+name_vec[3]+".jpg"
        src=img_root+real_img_name
        caching=true
        loader.load(src, callback(image_id, false))
    }
    is_traj_caching=false
}
function show_path(){
    var path_data=[]
    for (var i =0; i<cur_path.length; i++){
//        if (cur_path[i][2] in leaflet_objs){
            path_data.push([cur_path[i][1], cur_path[i][0]])
//        }
    }
    cur_path_obj = L.polyline(path_data, {color: 'Grey', interactive:false}).addTo(mymap)
}
async function play_thread(){
    if (cur_play_pos<0 || cur_play_pos>=cur_path.length){
        return
    }
    pause_btn=document.getElementById('pause_btn')
    is_pasue=false
    pause_btn.innerHTML="Pause"
    for (var i =cur_play_pos; i<cur_path.length; i++){
        if (stop_playing_flag==true){
            stop_playing_flag=false
            break
        }
        cur_play_pos=i
        while(!show_img(cur_path[i][2])){
            await sleep(100)
        }
        move_cur_img_marker([cur_path[i][1], cur_path[i][0]])
        await sleep(200)
    }
    is_pasue=true
    pause_btn.innerHTML="Resume"
}
function play(){
    if (cur_sel_img==""){
        return
    }
    if(is_pasue==false){
        return
    }
    cur_play_pos=-1
    for (var i=0; i<cur_path.length; i++){
        if (cur_sel_img==cur_path[i][2]){
            cur_play_pos=i
        }
    }
    if (cur_play_pos==-1){
        return
    }
    play_thread()
    is_pasue=false
}
function pause(){
    if (is_pasue){
        play_thread()
    }else{
        stop_playing_flag=true
    }
}
function cal_path(){
    if (is_traj_caching==true){
        stop_caching_flag=true
    }
    if (s_img=="" || d_img==""){
        return
    }
    if (cur_path_obj!=null){
        mymap.removeLayer(cur_path_obj)
        cur_path_obj=null
    }
    $.ajax({
        type: 'GET',
        url: '../../get_route_by_graph',
        data: {start_id:s_img, dst_id:d_img},
        dataType: 'json',
        success: function(data) {
            if (data.length>0){
                cur_path=data
                show_path()
                cache_img()
                cur_sel_img=cur_path[0][2]
                show_img(cur_sel_img)
                move_cur_img_marker([cur_path[0][1], cur_path[0][0]])
            }
        },
    });
}
function stop_cache(){
    if (is_traj_caching==true){
        stop_caching_flag=true
    }
}
function output(){
    video_name=$("#output_name").val()
    if (video_name==""){
        return
    }
    if (s_img=="" || d_img==""){
        return
    }
    $.ajax({
        type: 'GET',
        url: '../../add_output_task',
        data: {start_id:s_img, end_id:d_img, name:video_name},
        dataType: 'json',
        success: function(data) {
            
        },
    });
}
$(document).ready(function(){
    $.ajaxSetup({
        async: false
    });
    show_map()
    fetch_traj_list(10, [118, 32])
    show_img_gpss()
    init_pano()
    animate()
})

