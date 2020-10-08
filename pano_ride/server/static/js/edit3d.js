var mymap
var orbitControls
var renderer
var scene
var camera
var raycaster
var mouse
var clock
var frames=null
var graphs=null
var mps=null
var traj_threejs=[]
var count_rend_point=0
var node_threejs=[]
var edge_threejs=[]
var mp_threejs=[]
var tmp_line_threejs=[]
var first_choose_pt=[]
var view_mode=-1
var scale_candi=-1
var choose_pt_threejs=[]
var choose_edge_threejs=[]
var tran_ori_frame=-1
var del_nodes=[]
var del_edges=[]
var add_edge_node1=-1
var add_edge_node2=-1
var point_size=1
var pick_thres=0.03
var camera_pano, scene_pano, renderer_pano
var pano_material
var img_file_list={}
var loader
var dir_marker_threejs=null
var cur_frame_id=null
var isUserInteracting = false,
    lon = 0, lat = 0,
    phi = 0, theta = 0,
    distance = 1.1,
    onPointerDownPointerX = 0,
    onPointerDownPointerY = 0,
    onPointerDownLon = 0,
    onPointerDownLat = 0;

function add_threejs_pts(pts, color, size){
    var pointMaterial =  new THREE.PointsMaterial({
        size: size,
        color: color,
        sizeAttenuation: false
    });
    var tmp_geo = new THREE.Geometry();
    var tmp_mat = pointMaterial;
    for (var i=0; i<pts.length; i++) {
        count_rend_point=count_rend_point+1
        tmp_geo.vertices.push(new THREE.Vector3(pts[i][0], pts[i][2], -pts[i][1]));
    }
    var tmp_threejs_pts = new THREE.Points(tmp_geo, tmp_mat);
    traj_threejs.push(tmp_threejs_pts)
    scene.add(tmp_threejs_pts);
    return tmp_threejs_pts
}

function add_threejs_lines(pts,color){
    var points = []
    for (var i=0; i<pts.length; i++){
        count_rend_point=count_rend_point+1
        points.push( new THREE.Vector3(pts[i][0], pts[i][2], -pts[i][1]))
    }
    var trajGeometry = new THREE.Geometry()
    trajGeometry.setFromPoints( points )
    var trajLine = new THREE.Line( trajGeometry, new THREE.LineBasicMaterial({
        color: color, linewidth: 1, opacity: 1
    }));
    traj_threejs.push(trajLine)
    scene.add(trajLine);
    return trajLine
}

function add_threejs_segment(segs,color, addlist=true){
    points=[]
    for (var i=0; i<segs.length; i++){
        count_rend_point=count_rend_point+2
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
    scene.add(lineobj)
    return lineobj
}


async function init_3d() {
    scene = new THREE.Scene()
    mouse = new THREE.Vector2()
    clock = new THREE.Clock()
    var container=document.getElementById("container")
    var sceneColor = new THREE.Color( 0x888888 );
    scene.background = sceneColor.setStyle(color="rgb(255,255,255)");
    var divHeight = window.innerHeight;
    var divWidth = window.innerWidth;
    //camera = new THREE.PerspectiveCamera(25, window.innerWidth / window.innerHeight, 0.1, 10000);
    camera = new THREE.OrthographicCamera( divWidth/-50, divWidth/50, divHeight/50, divHeight/-50, 0.1, 10000 )
    camera.position.set(0, 0, 500);
    camera.up.x = 0;
    camera.up.y = 0;
    camera.up.z = 1;
    camera.lookAt(0, 0, 0);
    camera.aspect = divWidth/divHeight;
    camera.updateProjectionMatrix();
    renderer = new THREE.WebGLRenderer();
    renderer.setClearColor(new THREE.Color(0x000000));
    container.appendChild(renderer.domElement);
    renderer.setSize( divWidth, divHeight );
    window.addEventListener( 'resize', onWindowResize, false );
    orbitControls = new THREE.OrbitControls( camera, renderer.domElement )
    orbitControls.maxPolarAngle=3.1415926*3/4
    var divisions0 = 10;
    var geometry0 = new THREE.Geometry();
    var size0 = 1000;
    geometry0.vertices.push(new THREE.Vector3(-Math.ceil(size0 / 2), 0, 0));
    geometry0.vertices.push(new THREE.Vector3(Math.ceil(size0 / 2), 0, 0));
    for (var i = 0, len = Math.ceil(size0 / divisions0); i <= len; i++) {
        var line0 = new THREE.Line(geometry0, new THREE.LineBasicMaterial({
            color: 0xEEEEEE, opacity: 0.1
        }));
        line0.position.y = i * divisions0 - Math.ceil(size0 / 2);
        scene.add(line0);
        var line00 = new THREE.Line(geometry0, new THREE.LineBasicMaterial({
            color: 0xEEEEEE, opacity: 0.1
        }));
        line00.position.x = i * divisions0 - Math.ceil(size0 / 2);
        line00.rotation.z = Math.PI / 2;
        scene.add(line00);
    }
    
    var geometry_x = new THREE.Geometry();
    var xaxis_length = 600.0;
    geometry_x.vertices.push(new THREE.Vector3(0, 0, 1));
    geometry_x.vertices.push(new THREE.Vector3(xaxis_length, 0, 1));
    var line_x = new THREE.Line(geometry_x, new THREE.LineBasicMaterial({
        color: 0xFFAAAA, linewidth: 1, opacity: 0.1
    }));
    line_x.position.set(-1.0*xaxis_length/2, 0, 0) ;
    scene.add(line_x);
    var geometry_y = new THREE.Geometry();
    var yaxis_length = 600.0;
    geometry_y.vertices.push(new THREE.Vector3(0, 0, 1));
    geometry_y.vertices.push(new THREE.Vector3(0, yaxis_length, 1));
    var line_y = new THREE.Line(geometry_y, new THREE.LineBasicMaterial({
         color: 0xAAFFAA, linewidth: 1, opacity: 0.1
    }));
    line_y.position.set(0, -1.0*yaxis_length/2, 0);
    scene.add(line_y);
    var geometry_z = new THREE.Geometry();
    var axis_length = 600.0;
    geometry_z.vertices.push(new THREE.Vector3(0, 0, 0));
    geometry_z.vertices.push(new THREE.Vector3(0, 0, axis_length/2));
    var line_z = new THREE.Line(geometry_z, new THREE.LineBasicMaterial({
        color: 0xAAAAFF, linewidth: 1, opacity: 0.1
    }));
    line_z.position.set(0, 0, 0) ;
    scene.add(line_z);
    var controls2 = new function () {
        this.View = 'normal';
        this.SwitchCamera = function () {
            if (view_mode == -1) {
                camera.position.x = 0
                camera.position.y = 0
                camera.position.z = 3500
                camera.lookAt(0, 0, 0)
                view_mode=0
            } else if (view_mode == 0) {
                camera.position.x = 3500
                camera.position.y = 0
                camera.position.z = 0
                camera.lookAt(0, 0, 0)
                view_mode=1
            } else if (view_mode == 1) {
                camera.position.x = 0
                camera.position.y = 3500
                camera.position.z = 0
                camera.lookAt(0, 0, 0)
                view_mode=-1
            }
        }
    }
    var gui2 = new dat.GUI({ autoPlace: false, width: 90 });
    gui2.add(controls2, 'SwitchCamera').listen();
    gui2.domElement.id = "camera_gui";
    container.appendChild(gui2.domElement);
    $('span.property-name').hide();
    $(".button").append("SwitchView");
    gui2.closed = true; // hide menu bar
    raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = pick_thres
    raycaster.linePrecision = pick_thres
    stats = new Stats();
    container.appendChild( stats.dom );
    renderer.domElement.addEventListener("click", onclick, true);
    renderScene()
}

function renderScene(){
    orbitControls.update()
    requestAnimationFrame(renderScene)
    renderer.render(scene, camera)
    stats.update()
    update_pano()
}
function onWindowResize(){
    var divHeight = window.innerHeight
    var divWidth = window.innerWidth
    camera.aspect = divWidth/divHeight
    camera.updateProjectionMatrix()
    renderer.setSize( divWidth, divHeight)
}
function clear_3d_scene(){
    for (var i=0; i<traj_threejs.length; i++){
        scene.remove(traj_threejs[i])
    }
    traj_threejs=[]
    mp_threejs=[]
    tmp_line_threejs=[]
    node_threejs=[]
    edge_threejs=[]
}

function update_scene(){
    if (mps==null || frames==null || graphs==null){
        return
    }
    var all_mps=[]
    for(var frame_id in mps){
        sub_mps=mps[frame_id]
        for (var i=0; i<sub_mps.length; i++){
            all_mps.push(sub_mps[i])
        }
    }
    var color="#1b004a"
    mp_threejs.push(add_threejs_pts(all_mps, color, point_size))
    var all_nodes=[]
    var all_edges=[]
    for (var i=0; i<graphs["node"].length; i++){
        tmp_frameid = graphs["node"][i]["id"]
        all_nodes.push(frames[tmp_frameid][1])
    }
    for (var i=0; i<graphs["conn"].length; i++){
        tmp_conn = graphs["conn"][i]
        v1_id=graphs["node"][tmp_conn["v1"]]["id"]
        v2_id=graphs["node"][tmp_conn["v2"]]["id"]
        edge=[]
        edge.push(frames[v1_id][1])
        edge.push(frames[v2_id][1])
        all_edges.push(edge)
    }
    color="#02f07a"
    node_threejs.push(add_threejs_pts(all_nodes, color, 5))
    edge_threejs.push(add_threejs_segment(all_edges, color))
}

function clear_tmp_line(){
    if (tmp_line_threejs.length>0){
        for(var i=0; i<tmp_line_threejs.length; i++){
            scene.remove(tmp_line_threejs[i])
        }
        tmp_line_threejs=[]
    }
}

function show_image(img_file){
    var reader = new FileReader();
    reader.onload = function (e) {
        loader.load(e.target.result, show_img_callback());
    };
    reader.readAsDataURL(img_file);
}

function show_img_callback() {
    return (texture) => {
//        texture.generateMipmaps = false;
//        texture.wrapS = texture.wrapT = THREE.ClampToEdgeWrapping;
//        texture.minFilter = THREE.LinearFilter;
        pano_material.map=texture
        pano_material.map.needsUpdate=true
        
    }
}

function clear_choosed_pt(){
    if (choose_pt_threejs.length>0){
        for(var i=0; i<choose_pt_threejs.length; i++){
            scene.remove(choose_pt_threejs[i])
        }
        choose_pt_threejs=[]
    }
}

function multi_v(v, s){
    return [v[0]*s, v[1]*s, v[2]*s]
}

function sub_v(v1, v2){
    return [v1[0]-v2[0],v1[1]-v2[1],v1[2]-v2[2]]
}

function add_v(v1, v2){
    return [v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2]]
}

function ok_btn(){
    var b_scale= document.getElementById("rad_scale").checked
    var b_gravity= document.getElementById("rad_gravity").checked
    var b_resample= document.getElementById("rad_resample").checked
    var b_del_node= document.getElementById("rad_del_node").checked
    var b_del_conn= document.getElementById("rad_del_conn").checked
    var b_add_conn= document.getElementById("rad_add_conn").checked
    var b_tran_traj= document.getElementById("rad_tran_traj").checked
    var b_point_size= document.getElementById("rad_point_size").checked
    var b_pick_thres= document.getElementById("rad_pick_thres").checked
    if (b_scale && scale_candi>0){
        var scale_meter = parseFloat(document.getElementById("scale_input").value)
        scale_candi=scale_meter/scale_candi
        for(var frame_id in mps){
            sub_mps=mps[frame_id]
            for (var i=0; i<sub_mps.length; i++){
                sub_mps[i] = multi_v(sub_mps[i], scale_candi)
            }
        }
        for(var i=0; i<frames.length; i++){
            frames[i][1]=multi_v(frames[i][1], scale_candi)
        }
        clear_3d_scene()
        update_scene()
        scale_candi=-1
    }else if(b_gravity){
        var gravity_x_step = parseFloat(document.getElementById("gravity_x_input").value)/180*3.1415926
        var gravity_y_step = parseFloat(document.getElementById("gravity_y_input").value)/180*3.1415926
        var cos_x = Math.cos(gravity_x_step)
        var sin_x = Math.sin(gravity_x_step)
        var cos_y = Math.cos(gravity_y_step)
        var sin_y = Math.sin(gravity_y_step)
        for(var frame_id in mps){
            sub_mps=mps[frame_id]
            for (var i=0; i<sub_mps.length; i++){
                if (gravity_x_step!=0){
                    rot_x = cos_x*sub_mps[i][0]-sin_x*sub_mps[i][1]
                    rot_y = sin_x*sub_mps[i][0]+cos_x*sub_mps[i][1]
                    sub_mps[i][0]=rot_x
                    sub_mps[i][1]=rot_y
                }
                if (gravity_y_step!=0){
                    rot_x = cos_y*sub_mps[i][2]-sin_y*sub_mps[i][1]
                    rot_y = sin_y*sub_mps[i][2]+cos_y*sub_mps[i][1]
                    sub_mps[i][2]=rot_x
                    sub_mps[i][1]=rot_y
                }
            }
        }
        for(var i=0; i<frames.length; i++){
            if (gravity_x_step!=0){
                rot_x = cos_x*frames[i][1][0]-sin_x*frames[i][1][1]
                rot_y = sin_x*frames[i][1][0]+cos_x*frames[i][1][1]
                frames[i][1][0]=rot_x
                frames[i][1][1]=rot_y
            }
            if (gravity_y_step!=0){
                rot_x = cos_y*frames[i][1][2]-sin_y*frames[i][1][1]
                rot_y = sin_y*frames[i][1][2]+cos_y*frames[i][1][1]
                frames[i][1][2]=rot_x
                frames[i][1][1]=rot_y
            }
        }
        clear_3d_scene()
        update_scene()
    }else if(b_resample){
        var sample_step = parseFloat(document.getElementById("rad_resample_step_input").value)
        var last_pt=null
        var remain_frame_id=[]
        for (var i=0; i<frames.length; i++){
            if(last_pt==null){
                last_pt=frames[i][1]
            }
            var tmp_dist = cal_dist(last_pt, frames[i][1])
            if (tmp_dist<0.1){
                continue
            }
            remain_frame_id.push(i)
            last_pt=frames[i][1]
        }
        conns=[]
        nodes=[]
        for (var i=0; i<remain_frame_id.length; i++){
            nodes.push({"id":remain_frame_id[i]})
            if (i>0){
                conns.push({"v1":i-1,"v2":i})
            }
        }
        graphs={"conn":conns,"node":nodes}
        clear_3d_scene()
        update_scene()
    }else if(b_tran_traj){
        var tran_angle_step = parseFloat(document.getElementById("tran_angle_input").value)/180*3.1415926
        var tran_scale_step = parseFloat(document.getElementById("tran_scale_input").value)
        if (tran_ori_frame<0){
            return
        }
        var cos_z = Math.cos(tran_angle_step)
        var sin_z = Math.sin(tran_angle_step)
        ori_posi=frames[tran_ori_frame][1]
        for (var i=tran_ori_frame; i<frames.length; i++){
            local_posi_tmp = sub_v(frames[i][1], ori_posi)
            if (tran_scale_step!=1){
                frames[i][1]=add_v(multi_v(local_posi_tmp, tran_scale_step), ori_posi)
            }
            if (tran_angle_step!=0){
                rot_x = cos_z*local_posi_tmp[0]-sin_z*local_posi_tmp[2]
                rot_z = sin_z*local_posi_tmp[0]+cos_z*local_posi_tmp[2]
                frames[i][1][0]=rot_x+ori_posi[0]
                frames[i][1][2]=rot_z+ori_posi[2]
                frames[i][2]=frames[i][2]+tran_angle_step*180/3.1415926
            }
        }
        for(var frame_id in mps){
            if (frame_id<tran_ori_frame){
                continue
            }
            sub_mps=mps[frame_id]
            for (var i=0; i<sub_mps.length; i++){
                local_posi_tmp = sub_v(sub_mps[i], ori_posi)
                if (tran_scale_step!=1){
                    sub_mps[i]=add_v(multi_v(local_posi_tmp, tran_scale_step), ori_posi)
                }
                if (tran_angle_step!=0){
                    rot_x = cos_z*local_posi_tmp[0]-sin_z*local_posi_tmp[2]
                    rot_z = sin_z*local_posi_tmp[0]+cos_z*local_posi_tmp[2]
                    sub_mps[i][0]=rot_x+ori_posi[0]
                    sub_mps[i][2]=rot_z+ori_posi[2]
                }
            }
        }
        clear_3d_scene()
        update_scene()
        clear_choosed_pt()
        choose_pt_threejs.push(add_threejs_pts([ori_posi], "#ff0000", 10))
    }else if(b_del_node){
        if (del_nodes.length==0){
            return
        }
        conns=[]
        nodes=[]
        node_id_old_new_table=[]
        for (var i=0; i<graphs["node"].length; i++){
            if (!del_nodes.includes(i)){
                nodes.push(graphs["node"][i])
                node_id_old_new_table.push(nodes.length-1)
            }else{
                node_id_old_new_table.push(-1)
            }
        }
        for (var i=0; i<graphs["conn"].length; i++){
            if (!del_nodes.includes(graphs["conn"][i]["v1"]) && !del_nodes.includes(graphs["conn"][i]["v2"])){
                var new_v1=node_id_old_new_table[graphs["conn"][i]["v1"]]
                var new_v2=node_id_old_new_table[graphs["conn"][i]["v2"]]
                conns.push({"v1":new_v1,"v2":new_v2})
            }
        }
        graphs={"conn":conns,"node":nodes}
        clear_3d_scene()
        update_scene()
        del_nodes=[]
    }else if(b_del_conn){
        if (del_edges.length==0){
            return
        }
        conns=[]
        for (var i=0; i<graphs["conn"].length; i++){
            if (!del_edges.includes(i)){
                conns.push(graphs["conn"][i])
            }
        }
        graphs["conn"]=conns
        clear_3d_scene()
        update_scene()
        del_edges=[]
    }else if(b_add_conn){
        if (add_edge_node1>0 && add_edge_node2>0){
            graphs["conn"].push({"v1":add_edge_node1, "v2":add_edge_node2})
            clear_3d_scene()
            update_scene()
            add_edge_node1=-1
            add_edge_node2=-1
        }
    }else if(b_point_size){
        point_size = parseFloat(document.getElementById("point_size_input").value)
        clear_3d_scene()
        update_scene()
    }else if(b_pick_thres){
        pick_thres = parseFloat(document.getElementById("pick_thres_input").value)
        raycaster = new THREE.Raycaster();
        raycaster.params.Points.threshold = pick_thres
        raycaster.linePrecision = pick_thres
    }
}

function save_file(obj, name){
    var tmp_str=JSON.stringify(obj)
    var a = document.createElement("a")
    var file = new Blob([tmp_str], {type: "json"});
    a.href = URL.createObjectURL(file);
    a.download = name;
    a.style.display = "none"
    a.click()
}

function save_btn(){
    save_file(mps, "mps_seg.json")
    save_file(frames, "frames.json")
    save_file(graphs, "graph.json")
}

function update_dir_marker(){
    if (cur_frame_id==null){
        return
    }
    var img_posi=frames[cur_frame_id][1]
    if (dir_marker_threejs!=null){
        scene.remove(dir_marker_threejs)
        dir_marker_threejs=null
    }
    var rad_angle=lon/180*3.1415926
    var dir=[Math.sin(rad_angle)*1.2, 0,Math.cos(rad_angle)*1.2]
    var dir_vec=[img_posi,add_v(dir, img_posi)]
    dir_marker_threejs=add_threejs_segment([dir_vec],"#ff0000", false)
}

function onclick(event) {
    event.preventDefault();
    mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
    mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
    raycaster.setFromCamera(mouse, camera)
    if (node_threejs.length>0){
        var intersects = raycaster.intersectObjects(node_threejs, true)
        if (intersects.length > 0) {
            var pt_ind=intersects[0].index
            var tmp_p1= frames[graphs["node"][pt_ind]["id"]][1]
            var img_id=frames[graphs["node"][pt_ind]["id"]][0]
            show_image(img_file_list[img_id])
            cur_frame_id=graphs["node"][pt_ind]["id"]
            update_dir_marker()
            var b_scale= document.getElementById("rad_scale").checked
            if (b_scale){
                if (choose_pt_threejs.length>=2){
                    clear_choosed_pt()
                }
                choose_pt_threejs.push(add_threejs_pts([tmp_p1], "#ff0000", 10))
                if (first_choose_pt.length==0){
                    clear_tmp_line()
                    first_choose_pt=tmp_p1
                }else{
                    scale_candi = cal_dist(first_choose_pt, tmp_p1)
                    segs=[[first_choose_pt, tmp_p1]]
                    tmp_line_threejs.push(add_threejs_segment(segs,"#ff0012"))
                    first_choose_pt=[]
                }
            }
            var b_tran_traj= document.getElementById("rad_tran_traj").checked
            if (b_tran_traj){
                clear_choosed_pt()
                choose_pt_threejs.push(add_threejs_pts([tmp_p1], "#ff0000", 10))
                tran_ori_frame=graphs["node"][pt_ind]["id"]
            }
            var b_del_node= document.getElementById("rad_del_node").checked
            if (b_del_node){
                choose_pt_threejs.push(add_threejs_pts([tmp_p1], "#ff0000", 10))
                del_nodes.push(pt_ind)
            }
            var b_add_conn= document.getElementById("rad_add_conn").checked
            if (b_add_conn){
                if (choose_pt_threejs.length>=2){
                    clear_choosed_pt()
                }
                choose_pt_threejs.push(add_threejs_pts([tmp_p1], "#ff0000", 10))
                if (add_edge_node1<0 && add_edge_node2<0){
                    add_edge_node1=pt_ind
                    return
                }
                if (add_edge_node1>0 && add_edge_node2<0){
                    add_edge_node2=pt_ind
                    return
                }
                if (add_edge_node1>0 && add_edge_node2>0){
                    add_edge_node1=pt_ind
                    add_edge_node2=-1
                    return
                }
            }
            var b_pick_thres= document.getElementById("rad_pick_thres").checked
            if (b_pick_thres){
                clear_choosed_pt()
                choose_pt_threejs.push(add_threejs_pts([tmp_p1], "#ff0000", 10))
            }
        }
    }
    if (edge_threejs.length>0){
        var intersects = raycaster.intersectObjects(edge_threejs, true)
        if (intersects.length > 0) {
            var b_del_conn= document.getElementById("rad_del_conn").checked
            if (b_del_conn){
                pt_ind=Math.trunc(intersects[0].index/2)
                v1=graphs["conn"][pt_ind]["v1"]
                v2=graphs["conn"][pt_ind]["v2"]
                v1_posi=frames[graphs["node"][v1]["id"]][1]
                v2_posi=frames[graphs["node"][v2]["id"]][1]
                choose_pt_threejs.push(add_threejs_pts([v1_posi,v2_posi], "#ff0000", 10))
                del_edges.push(pt_ind)
            }
        }
    }
}

function cal_dist(v1, v2){
    dist=Math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1])+(v1[2]-v2[2])*(v1[2]-v2[2]))
    return dist
}

function process_folder(){
    var graph_file=null
    var mp_file=null
    var frame_file=null
    var voice_list_file=[]
    for (var file_id=0; file_id<this.files.length; file_id++){
        var file=this.files[file_id]
        var path=file["webkitRelativePath"]
        var path_vec = path.split("/")
        if (path_vec.length<2){
            continue
        }
        var file_name=path_vec[path_vec.length-1]
        if (file_name.includes(".jpg")){
            img_file_list[parseInt(file_name.split(".jpg")[0])-1]=file
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
        if (file_name.includes(".mp3")){
            mp3_list[file_name]=file
        }
    }
    if (frame_file!=null){
        var fr = new FileReader();
        fr.onload = function(e) {
            var lines = e.target.result;
            frames = JSON.parse(lines)
            update_scene()
        };
        fr.readAsText(frame_file);
    }
    if (graph_file!=null){
        var fr = new FileReader();
        fr.onload = function(e) {
            var lines = e.target.result;
            graphs = JSON.parse(lines)
            update_scene()
        };
        fr.readAsText(graph_file);
    }
    if (mp_file!=null){
        var fr = new FileReader();
        fr.onload = function(e) {
            var lines = e.target.result;
            mps = JSON.parse(lines)
            update_scene()
        };
        fr.readAsText(mp_file);
    }
}

function onPanoMouseOut(){
    isUserInteracting=false
}

function onPanoMouseDown( event ) {
    event.preventDefault();
    isUserInteracting = true;
    onPointerDownPointerX = event.clientX;
    onPointerDownPointerY = event.clientY;
    onPointerDownLon = lon;
    onPointerDownLat = lat;
}
function onPanoMouseMove( event ) {
    if ( isUserInteracting === true ) {
        lon = ( onPointerDownPointerX - event.clientX ) * 0.2 + onPointerDownLon;
        update_dir_marker()
        lat = ( event.clientY - onPointerDownPointerY ) * -0.2 + onPointerDownLat;
        if (lat>20){
           onPointerDownPointerY=onPointerDownPointerY-(lat-20)
           lat=20
        }
        if (lat<-27){
           onPointerDownPointerY=onPointerDownPointerY+(-27-lat)
           lat=-27
        }
    }
}
function onPanoMouseUp() {
    isUserInteracting = false
}

function update_pano() {
    if (frames==null || cur_frame_id==null){
        return
    }
    phi = THREE.Math.degToRad( 90 - lat );
    var img_dir=frames[cur_frame_id][2]
    theta = THREE.Math.degToRad( lon+img_dir-20);
    camera_pano.position.x = distance * Math.sin( phi ) * Math.cos( theta );
    camera_pano.position.y = distance * Math.cos( phi );
    camera_pano.position.z = distance * Math.sin( phi ) * Math.sin( theta );
    camera_pano.lookAt( camera_pano.target );
    renderer_pano.render( scene_pano, camera_pano );
}

function init_pano() {
    var container,container_query, mesh;
    container = document.getElementById('pano');
    container_query=$("#pano");
    var divWidth = container_query.width();
    var divHeight = container_query.height();
    var rate=divWidth/divHeight
    camera_pano = new THREE.PerspectiveCamera( 60, rate, 1, 1100 );
    camera_pano.target = new THREE.Vector3( 0, 0, 0 );
    scene_pano = new THREE.Scene();
    var geometry = new THREE.SphereBufferGeometry( 1000, 60, 40 );
    // invert the geometry on the x-axis so that all of the faces point inward
    geometry.scale( -1, 1, 1 );
    var texture =  new THREE.TextureLoader().load('#');
    pano_material = new THREE.MeshBasicMaterial( { map: texture } );
    mesh = new THREE.Mesh( geometry, pano_material );
    scene_pano.add( mesh );
    renderer_pano = new THREE.WebGLRenderer();
    renderer_pano.setPixelRatio( window.devicePixelRatio );
    renderer_pano.setSize( divWidth, divHeight);
    container.appendChild( renderer_pano.domElement );
    container_query.on( 'mousedown', onPanoMouseDown);
    container_query.on( 'mousemove', onPanoMouseMove);
    container_query.on( 'mouseup', onPanoMouseUp);
    container_query.on( 'mouseout', onPanoMouseOut);
    loader = new THREE.TextureLoader()
}

$(document).ready(function(){
   $.ajaxSetup({
       async: true
   })
   init_pano()
   init_3d()
   inputNode = document.getElementById("load_folder");
   inputNode.addEventListener('change', process_folder, false)
})

