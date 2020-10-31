var oss_root="https://ride-v.oss-cn-beijing.aliyuncs.com/"
var oss_raw_data="pano_maps"
var client = new OSS.Wrapper({
    region: "oss-cn-beijing",
    accessKeyId: "LTAI4GJDtEd1QXeUPZrNA4Yc",
    accessKeySecret: "rxWAZnXNhiZ8nemuvshvKxceYmUCzP",
    bucket: "ride-v"
});

var upload_list=[]
var loadfile_map=""
function show_map_list(){
    $.ajax({
        type: 'GET',
        url: '../../list_all_maps',
        dataType: 'json',
        success: function(data) {
            if (data.length>0){
                var str=""
                for(var i=0; i<data.length; i++){
                    var map_name=data[i]["name"]
                    var author=data[i]["author"]
                    var time=data[i]["time"]
                    var status=data[i]["status"]
                    str_upload='(<a href="#" onclick=upload_insv("'+map_name+'")>upload</a>)'
                    str_proc='(<a href="#" onclick=process_map("'+map_name+'")>process</a>)'
                    str_trim='(<a href="#" onclick=trim_map("'+map_name+'")>trim</a>)'
                    str=str+"<b>"+map_name+", "+author+", "+time+", "+status+"</b>"+str_upload+str_proc+"</br>"
                    for (var j=0; j<data[i]["insv"].length; j++){
                        var insv_name=data[i]["insv"][j]["name"]
                        var insv_size=(data[i]["insv"][j]["size"]/1024/1024/1024).toFixed(2)
                        var insv_time=data[i]["insv"][j]["time"]
                        var str_del='(<a href="#" onclick=del_insv("'+insv_name+'","'+map_name+'")>del</a>)'
                        file_oss_addr=oss_root+oss_raw_data+"/"+map_name+"/"+insv_name
                        var str_file_link='<a href="'+file_oss_addr+'">'+insv_name+'</a>'
                        str=str+"----"+str_file_link+", "+insv_size+" GB, "+insv_time+str_del+"</br>"
                    }
                }
                document.getElementById("map_root").innerHTML=str
            }
        },
    });
}

function create_map(){
    var name=document.getElementById("map_name_input").value
    var author=document.getElementById("author_input").value
    if (name!="" && author!=""){
        $.ajax({
            type: 'GET',
            url: '../../create_map',
            data: {name:name, author:author},
            dataType: 'json',
            success: function(data) {
                if (data[0]=="ok"){
                    show_map_list()
                }
            },
        });
    }
}

function upload_one_insv(task_id){
    var storeas = oss_raw_data+"/"+upload_list[task_id][0]+"/"+upload_list[task_id][1].name
    client.multipartUpload(storeas, upload_list[task_id][1], {
        parallel: 1,
        partSize: 1000 * 1024,
        timeout: 120000,
        progress: function* (percent, cpt)
        {
            upload_list[task_id][2]=(percent*100).toFixed(1)
            update_uploading_list()
        }
    }).then(function (result) {
        $.ajax({
            type: 'GET',
            url: '../../add_insv',
            data: {map_name:upload_list[task_id][0], insv:upload_list[task_id][1].name, insv_size:upload_list[task_id][1].size},
            dataType: 'json',
            success: function(data) {
                if (data[0]=="ok"){
                    show_map_list()
                    upload_list[task_id][3]="done"
                    update_uploading_list()
                }
            },
        });
    }).catch(function (err) {
        console.log(err)
    });
}

function del_insv(insv, map_name){
    $.ajax({
        type: 'GET',
        url: '../../del_insv',
        data: {map_name:map_name, insv:insv},
        dataType: 'json',
        success: function(data) {
            if (data[0]=="ok"){
                show_map_list()
            }
        },
    });
}

function process_map(map_name){
    $.ajax({
        type: 'GET',
        url: '../../process_map',
        data: {map_name:map_name},
        dataType: 'json',
        success: function(data) {
            if (data[0]=="ok"){
                show_map_list()
            }
        },
    });
}

function trim_map(map_name){
    $.ajax({
        type: 'GET',
        url: '../../trim_map',
        data: {map_name:map_name},
        dataType: 'json',
        success: function(data) {
            if (data[0]=="ok"){
                show_map_list()
            }
        },
    });
}

function process_folder(value){
    if (loadfile_map==""){
        return
    }
    for (var file_id=0; file_id<value.files.length; file_id++){
        var file=value.files[file_id]
        upload_task=[loadfile_map, file, 0, "pending"]
        upload_list.push(upload_task)
    }
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function upload_insv(map_name){
    loadfile_map=map_name
    document.getElementById("load_folder").click()
}

async function upload_thread(){
    while (true){
        if (upload_list.length>0){
            new_download=-1
            for (var i=0; i<upload_list.length; i++){
                if (upload_list[i][3]=="pending"){
                    new_download=i
                }
                if (upload_list[i][3]=="uploading"){
                    new_download=-1
                    break
                }
            }
            if (new_download>=0){
                upload_list[new_download][3]="uploading"
                upload_one_insv(new_download)
                update_uploading_list()
            }
        }
        await sleep(1000)
    }
}

function update_uploading_list(){
    var uploading_root=document.getElementById("uploading_root")
    var str=""
    for (var i=0; i<upload_list.length; i++){
        str=str+upload_list[i][1].name+"("+upload_list[i][2]+"%, "+upload_list[i][3]+")"+"</br>"
    }
    uploading_root.innerHTML=str
}

$(document).ready(function(){
    $.ajaxSetup({
        async: false
    });
    show_map_list()
    upload_thread()
})

