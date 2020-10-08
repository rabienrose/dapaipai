//import * as THREE from '../../build/three.module.js';

var oss_root="https://ride-v.oss-cn-beijing.aliyuncs.com/"
var oss_mid_data="maps/shangrila/middle-files/"
var oss_config="maps/shangrila/video_seg.json"
var video_list=[]
var client = new OSS.Wrapper({
    region: "oss-cn-beijing",
    accessKeyId: "LTAI4GJDtEd1QXeUPZrNA4Yc",
    accessKeySecret: "rxWAZnXNhiZ8nemuvshvKxceYmUCzP",
    bucket: "ride-v"
});
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
var whole_bounding=null;
var leaflet_poly_list={}
var traj_posis_list={}
var cur_traj_posis=[]
var last_play_posi=-1
var play_video_dom=null
var leaflet_cur_node=null
var circleOpt= {
    radius: 5,
    color:"red",
    fill:true,
    fillColor:"red",
    fillOpacity:0.3
}

function show_map(){
    var map_root = document.getElementById("panel_1_1");
    var div_map = document.createElement("div");
    div_map.id="map";
    map_root.appendChild(div_map);
    mymap = L.map('map').setView([51.505, -0.09], 18);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 22,
        maxNativeZoom: 19
    }).addTo(mymap);
    L.control.scale({imperial:false}).addTo(mymap)
    leaflet_cur_node=new L.CircleMarker ([51.505, -0.09], circleOpt).addTo(mymap)
}

function display_list(){
    var iframe = document.getElementById("list_frame");
    var elmnt = iframe.contentWindow.document.getElementById("list_root");
    var html_str='<table border="0">'
    video_list.sort(function(a, b){return a["order"] - b["order"]});
    for (var index = 0; index < video_list.length; index++)
    {
        html_str=html_str+"<tr>"
        html_str=html_str+'<td onclick="parent.click_item('+index+')">'+video_list[index]["name"]+"</td>"
        html_str=html_str+"<td>"+'<input type="text" value='+video_list[index]["order"]+' class="order_box">'+"</td>"
        html_str=html_str+"<td>"+'<input type="text" value='+video_list[index]["start"]+' class="order_box1">'+"</td>"
        html_str=html_str+"<td>"+'<input type="text" value='+video_list[index]["end"]+' class="order_box1">'+"</td>"
        html_str=html_str+"</tr>"
    }
    html_str=html_str+"</table>"
    elmnt.innerHTML = html_str;
}

function unselect_all(){
    var iframe = document.getElementById("list_frame");
    var elmnt = iframe.contentWindow.document.getElementById("list_root");
    video_len=elmnt.childNodes[0].rows.length;
    for(var i=0; i<video_len; i=i+1) {
        elmnt.childNodes[0].rows[i].cells[0].style.color = "#000000"
    }
}

function click_traj(e){
    console.log(e.target.video)
    var id = get_video_info(e.target.video)
    choose_video(id)
}

function get_selected_row(){
    var iframe = document.getElementById("list_frame");
    var elmnt = iframe.contentWindow.document.getElementById("list_root");
    video_len=elmnt.childNodes[0].rows.length;
    for(var i=0; i<video_len; i=i+1) {
        item_color = elmnt.childNodes[0].rows[i].cells[0].style.color
        if(item_color=='rgb(255, 0, 0)'){
            return elmnt.childNodes[0].rows[i]
        }
    }
    return null
}

function get_selected(){
    var sel_row=get_selected_row()
    if(sel_row!=null){
        return sel_row.cells[0].innerHTML
    }else{
        return ""
    }
}

function set_frame_time(video_time, type){
    var sel_row=get_selected_row()
    if(sel_row!=null){
        if (type=="begin"){
            sel_row.cells[2].childNodes[0].value=video_time
        }else{
            sel_row.cells[3].childNodes[0].value=video_time
        }
    }
}

function get_begin_end(){
    var sel_row=get_selected_row()
    var begin_time=-1
    var end_time=-1
    if(sel_row!=null){
        begin_time=parseFloat(sel_row.cells[2].childNodes[0].value)
        end_time=parseFloat(sel_row.cells[3].childNodes[0].value)
    }
    return [begin_time, end_time]
}

function move_to_traj(video_name){
    temp_poly=leaflet_poly_list[video_name]
    mymap.fitBounds(temp_poly.getBounds())
}

function choose_video(ind){
    unselect_all()
    var iframe = document.getElementById("list_frame");
    var elmnt = iframe.contentWindow.document.getElementById("list_root");
    elmnt.childNodes[0].rows[ind].cells[0].style.color = "#ff0000"
}

function playSelectedFile(video_name, video_dom, time_set) {
    video_dom.src="https://ride-v.oss-cn-beijing.aliyuncs.com/maps/shangrila/middle-files/"+video_name+"/video.mp4"
    video_dom.load()
    //video.src_name=video_name
    video_dom.currentTime=time_set
    video_dom.playbackRate=0.2
}

function play(){
    var video_name = get_selected()
    if (video_name!=null){
        play_video_dom = document.getElementById( 'video_pano' )
        play_video_dom.ontimeupdate = function() {play_update()}
        play_video_dom.onended = function() {play_end()}
        cur_traj_posis=traj_posis_list[video_name]
        show("pano")
    }
}

function speed_up(){
    change_playrate(1.1)
}

function speed_down(){
    change_playrate(0.9)
}

function change_playrate(val){
    var video_dom = document.getElementById( 'video_pano' )
    video_dom.playbackRate=video_dom.playbackRate*val
}

function step(val,type){
    var video_dom=null
    if (type=="begin"){
        video_dom= document.getElementById("start_video")
    }else if(type=="end"){
        video_dom= document.getElementById("end_video")
    }
    var time_set=video_dom.currentTime+val*0.03333333
    if (time_set>video_dom.duration){
        time_set=video_dom.duration
    }
    if (time_set<0){
        time_set=0
    }
    video_dom.currentTime=time_set
}

function click_item(video_ind){
    choose_video(video_ind)
    move_to_traj(video_list[video_ind]['name'])
}

function set_frame(type){
    var video_dom=null
    if (type=="begin"){
        video_dom= document.getElementById("start_video")
    }else if(type=="end"){
        video_dom= document.getElementById("end_video")
    }
    set_frame_time(video_dom.currentTime, type)
}

function show(type){
    video_name = get_selected()
    if (video_name==""){
        return
    }
    [begin_time, end_time] = get_begin_end()
    var video_dom=null
    var time_set=end_time
    if (type=="begin"){
        video_dom= document.getElementById("start_video")
        time_set=begin_time
    }else if(type=="end"){
        video_dom= document.getElementById("end_video")
    }else if(type=="pano"){
        video_dom = document.getElementById( 'video_pano' )
        time_set=0
    }
    playSelectedFile(video_name, video_dom, time_set)
}

function set_cur_posi_on_map(latlon){
    var currentCircleCoords = L.latLng (latlon[0], latlon[1])
    leaflet_cur_node.setLatLng (currentCircleCoords);
}

function play_end(){
    var video_name = get_selected()
    var cur_id = get_video_info(video_name)
    if (cur_id<video_list.length-1){
        choose_video(cur_id+1)
        play()
    }
}

function play_update() {
    if(play_video_dom.paused==true){
        return
    }
    if (Math.abs(last_play_posi-play_video_dom.currentTime)>0.3){
        last_play_posi=play_video_dom.currentTime
        var percent=last_play_posi/play_video_dom.duration
        var max_len=cur_traj_posis.length
        var temp_count=Math.floor(percent*max_len)
        var temp_posi=cur_traj_posis[temp_count]
        set_cur_posi_on_map(temp_posi)
    }
}

function show_traj(video_name){
    var url = client.signatureUrl(oss_mid_data+video_name+"/frame_info.json");
    $.getJSON(url, function(data) {
        
        var latlngs = [];
        for (var i=0; i<data.length; i=i+1){
            latlngs.push([data[i][0], data[i][1]])
        }
        var color;
        var r = Math.floor(Math.random() * 255);
        var g = Math.floor(Math.random() * 255);
        var b = Math.floor(Math.random() * 255);
        color= "rgb("+r+" ,"+g+","+ b+")";
        
        var polyline = L.polyline(latlngs, {color: color}).addTo(mymap);
        if (whole_bounding==null){
            whole_bounding=L.latLngBounds(polyline.getBounds().getSouthWest(), polyline.getBounds().getNorthEast())
        }else{
            whole_bounding.extend(polyline.getBounds())
        }
        leaflet_poly_list[video_name]=polyline
        traj_posis_list[video_name]=latlngs
        mymap.fitBounds(whole_bounding);
        polyline.on('click', click_traj)
        polyline.video=video_name
    });
}

function show_all_trajs(){
    for (var i=0; i<video_list.length; i++){
        show_traj(video_list[i]["name"])
    }
}

function get_video_info(video_name){
    for (var i=0; i<video_list.length; i++){
        if (video_list[i]["name"]==video_name){
            return i
        }
    }
    return -1
}

async function fetch_data(){
    var url = client.signatureUrl(oss_config);
    $.getJSON(url, function(data) {
        video_list=data
    });

    var result = await client.list({
      prefix: oss_mid_data
    });
    for (var i=0; i<result.objects.length; i++){
        var obj=result.objects[i]
        splited_str = obj.name.split("/")
        if (splited_str[splited_str.length-1]=="frame_info.json"){
            video_name = splited_str[splited_str.length-2]
            if (get_video_info(video_name)==-1){
                video_list.push({
                "name":video_name,
                "order":-1,
                "start":-1,
                "end":-1
                            })
            }
        }
    }
    display_list()
    show_all_trajs()
}

async function putObject (text) {
  try {
    //var file = new Blob([text])
    data = new OSS.Buffer(text)
    let result = await client.put(oss_config, data);
    //console.log(result);
  } catch (e) {
    console.log(e);
  }
}

function save(){
    var iframe = document.getElementById("list_frame");
    var elmnt = iframe.contentWindow.document.getElementById("list_root");
    video_len=elmnt.childNodes[0].rows.length;
    for(var i=0; i<video_len; i=i+1) {
        var order = elmnt.childNodes[0].rows[i].cells[1].childNodes[0].value
        var start_frame = elmnt.childNodes[0].rows[i].cells[2].childNodes[0].value
        var end_frame = elmnt.childNodes[0].rows[i].cells[3].childNodes[0].value
        var video_name=elmnt.childNodes[0].rows[i].cells[0].innerHTML
        var index = get_video_info(video_name)
        if (index!=-1){
            video_list[index]["end"]=end_frame
            video_list[index]["start"]=start_frame
            video_list[index]["order"]=order
        }else{
            console.log("get_video_info error!!!")
        }
        
    }
    var file_content = JSON.stringify(video_list)
    putObject (file_content)
    display_list()
}

function init() {
    var container,container_query, mesh;
    container = document.getElementById( 'panel_2_3_1' );
    container_query=$("#panel_2_3_1");
    var w = container_query.width();
    var h = container_query.height();
    console.log("w: "+w+"   "+"h: "+h)
    var rate=w/h
    camera = new THREE.PerspectiveCamera( 50, rate, 1, 1100 );
    camera.target = new THREE.Vector3( 0, 0, 0 );
    scene = new THREE.Scene();
    var geometry = new THREE.SphereBufferGeometry( 1000, 60, 40 );
    // invert the geometry on the x-axis so that all of the faces point inward
    geometry.scale( -1, 1, 1 );
    var video = document.getElementById( 'video_pano' )
    var texture = new THREE.VideoTexture( video );
    var material = new THREE.MeshBasicMaterial( { map: texture } );
    mesh = new THREE.Mesh( geometry, material );
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
        lon = ( onPointerDownPointerX - event.clientX ) * 0.5 + onPointerDownLon;
        lat = ( event.clientY - onPointerDownPointerY ) * -0.5 + onPointerDownLat;
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

$(document).ready(function(){
    init();
    animate();
    $.ajaxSetup({
        async: false
    });
    fetch_data()
    show_map()
    
})

