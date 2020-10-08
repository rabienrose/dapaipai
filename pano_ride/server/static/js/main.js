var checkpoint_temp;
var oss_root="https://ride-v.oss-cn-beijing.aliyuncs.com/"
var oss_raw_data=oss_root+"raw_files/"
var oss_map_data=oss_root+"video_files/"
var client = new OSS.Wrapper({
    region: "oss-cn-beijing",
    accessKeyId: "LTAI4GJDtEd1QXeUPZrNA4Yc",
    accessKeySecret: "rxWAZnXNhiZ8nemuvshvKxceYmUCzP",
    bucket: "ride-v"
});
var tempCheckpoint

function upload_insv() {
    upload_file("insv")
}

function upload_mp4() {
    upload_file("MP4")
}

function show_list(data){
    //console.log(data)
    var html_str='<table border="1">'
    for (var index = 0; index < data.length; index++)
    {
        html_str=html_str+"<tr>"
        html_str=html_str+"<td>"+data[index]['name']+"</td>"
        if (data[index]['insv']!=""){
            html_str=html_str+"<td>"+"<a href="+oss_raw_data+data[index]['name']+".insv>"+data[index]['insv']+"</a>"+"</td>"
        }else{
            html_str=html_str+"<td>None</td>"
        }
        html_str=html_str+"<td>"+'<a href=javascript:del_oss_item("'+data[index]['name']+'")>delete</a>'+"</td>"
        if (data[index]['process']=="process"){
            html_str=html_str+"<td>"+'<a href=javascript:request_proc("'+data[index]['name']+'")>process</a>'+"</td>"
        }else{
            html_str=html_str+"<td>"+data[index]['process']+"</td>"
        }
        html_str=html_str+"<td>"+'<a href=javascript:clear_ws("'+data[index]['name']+'")>clear</a>'+"</td>"
        html_str=html_str+"</tr>"
    }
    html_str=html_str+"</table>"
    document.getElementById('raw_list').innerHTML = html_str;
}

function request_proc(name){
    
    extract_type=document.getElementById("extract_type").value
    console.log(extract_type)
    if (extract_type!='jpg' && extract_type!='png'){
        return;
    }
    jpg_quality=parseInt(document.getElementById("jpg_quality").value)
    console.log(jpg_quality)
    if (jpg_quality<2 || jpg_quality>31){
        return;
    }
    frame_per_meter=parseFloat(document.getElementById("frame_per_meter").value)
    console.log(frame_per_meter)
    if (frame_per_meter<0.1 || frame_per_meter>10){
        return;
    }
    out_resolution=parseInt(document.getElementById("out_resolution").value)
    console.log(out_resolution)
    if (out_resolution<720 || out_resolution>2000){
        return;
    }
    out_quality=parseInt(document.getElementById("out_quality").value)
    console.log(out_quality)
    if (out_quality<2 || out_quality>31){
        return;
    }
    $.ajax({
        type: 'GET',
        url: '../../process_raw',
        data: { name: name,
                extract_type:extract_type,
                jpg_quality:jpg_quality,
                frame_per_meter:frame_per_meter,
                out_resolution:out_resolution,
                out_quality:out_quality
           },
        dataType: 'json',
        success: function(data) {
            show_raw_list()
        },
        async: true
    });
}

function del_oss_item(name){
    $.ajax({
        type: 'GET',
        url: '../../del_raw',
        data: { name: name},
        dataType: 'json',
        success: function(data) {
            show_raw_list()
        },
        async: true
    });
}

function del_map(name){
    $.ajax({
        type: 'GET',
        url: '../../del_map',
        data: { name: name},
        dataType: 'json',
        success: function(data) {
            show_map_list()
        },
        async: true
    });
}

function clear_ws(name){
    $.ajax({
        type: 'GET',
        url: '../../clear_ws',
        data: { name: name},
        dataType: 'json',
        success: function(data) {
            
            show_task_status()
        },
        async: true
    });
}

function stop_process(){
    $.ajax({
        type: 'GET',
        url: '../../stop_process',
        dataType: 'json',
        success: function(data) {
            show_raw_list()
        },
        async: true
    });
}



function show_task_status(){
    $.ajax({
        type: 'GET',
        url: '../../check_process_status',
        dataType: 'json',
        success: function(data) {
            document.getElementById('progress_action').innerHTML=data['action']
            document.getElementById('progress_rate').innerHTML=data['progress']
            document.getElementById('memory_rate').innerHTML=data['mem']
            document.getElementById('cpu_rate').innerHTML=data['cpu']
            document.getElementById('free_disk').innerHTML=data['disk']
            document.getElementById('err_info').innerHTML=data['err']
        },
        async: true
    });
}

function show_raw_list(){
    $.ajax({
        type: 'GET',
        url: '../../show_raw_list',
        dataType: 'json',
        success: function(data) {
            show_list(data)
        },
        async: true
    });
}

function show_maps(data){
    //console.log(data)
    var html_str='<table border="1">'
    for (var index = 0; index < data.length; index++)
    {
        html_str=html_str+"<tr>"
        html_str=html_str+"<td>"+data[index]['name']+"</td>"
        if ('length' in data[index]){
            html_str=html_str+"<td>"+data[index]['length']+"</td>"
        }else{
            html_str=html_str+"<td></td>"
        }
        html_str=html_str+"<td>"+'<a href=javascript:del_map("'+data[index]['name']+'")>delete</a>'+"</td>"
        html_str=html_str+"<td>"+"<a href="+oss_map_data+data[index]['name']+"/video.mp4>"+data[index]['size']+"</a>"+"</td>"
        html_str=html_str+"</tr>"
    }
    html_str=html_str+"</table>"
    document.getElementById('map_list').innerHTML = html_str;
}

function show_map_list(){
    $.ajax({
        type: 'GET',
        url: '../../show_map_list',
        dataType: 'json',
        success: function(data) {
            show_maps(data)
        },
        async: true
    });
}

function upload_file(type){
    var scene_name= document.getElementById("scene_id").value
    if (scene_name==""){
        return;
    }
    var file = document.getElementById(type).files[0];
    if (file==null){
        console.log("no file")
        return
    }
    var val = document.getElementById(type).value;
    var suffix = val.substr(val.indexOf(".")+1);
    if (suffix!=type){
        console.log("not file")
        return
    }
    var storeas = "raw_files/"+scene_name + "."+type;
    client.multipartUpload(storeas, file, {
        parallel: 1,
        partSize: 1000 * 1024,
        timeout: 120000,
        progress: function* (percent, cpt)
        {
          document.getElementById(type+"_p").innerHTML=" ("+Math.round(percent*1000)/10+"%)";
          console.log('cpt: ' + cpt.doneParts.length);
        }
    }).then(function (result) {
        show_raw_list()
    }).catch(function (err) {
        console.log(err);
    });
}

function fetch_list(){
    show_map_list()
    show_raw_list()
}

$(document).ready(function(){
    fetch_list()
})

