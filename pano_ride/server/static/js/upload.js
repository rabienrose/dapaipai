var oss_root="https://ride-v.oss-cn-beijing.aliyuncs.com/"
var oss_raw_data="maps/shangrila/insv/"
var client = new OSS.Wrapper({
    region: "oss-cn-beijing",
    accessKeyId: "LTAI4GJDtEd1QXeUPZrNA4Yc",
    accessKeySecret: "rxWAZnXNhiZ8nemuvshvKxceYmUCzP",
    bucket: "ride-v"
});

var uploaded_list=[]
var to_upload_list=[]
var is_uploading=false

function upload_list(){
    if (is_uploading==false && to_upload_list.length>0){
        upload_one_insv(to_upload_list[to_upload_list.length-1])
    }
}

function upload_one_insv(file){
    is_uploading=true
    var storeas = oss_raw_data+file.name;
    client.multipartUpload(storeas, file, {
        parallel: 1,
        partSize: 1000 * 1024,
        timeout: 120000,
        progress: function* (percent, cpt)
        {
          document.getElementById("progress").innerHTML="Remain "+to_upload_list.length+" files. ("+Math.round(percent*1000)/10+"%)";
        }
    }).then(function (result) {
        to_upload_list.length=to_upload_list.length-1;
        is_uploading=false
        fetch_list()
    }).catch(function (err) {
        console.log(err);
    });
}

function upload_file(){
    if (is_uploading==true){
        return
    }
    is_uploading=true
    var file_dom=document.getElementById("insv").files
    for(var i=0; i<file_dom.length; i++){
        var file_name=file_dom[i].name
        if ( file_name.indexOf(".insv") != -1){

            if (uploaded_list.includes(file_name)){
                continue
            }
            to_upload_list.push(file_dom[i])
        }
    }
    is_uploading=false
}

function display_list(){
    var html_str='<table border="1">'
    for (var index = 0; index < uploaded_list.length; index++)
    {
        html_str=html_str+"<tr>"
        html_str=html_str+"<td>"+uploaded_list[index]+"</td>"
        html_str=html_str+"<td>"+'<a href=javascript:del_item("'+uploaded_list[index]+'")>delete</a>'+"</td>"
        html_str=html_str+"</tr>"
    }
    html_str=html_str+"</table>"
    document.getElementById('raw_list').innerHTML = html_str;
}

async function fetch_list(){
    var result = await client.list({
      prefix: oss_raw_data
    });
    uploaded_list=[]
    for (var i=0; i<result.objects.length; i++){
        var obj=result.objects[i]
        if (obj.name.indexOf(".insv") != -1){
            splited_str = obj.name.split("/")
            uploaded_list.push(splited_str[splited_str.length-1])
        }
    }
    display_list()
}

async function del_item(name){
    await client.delete(oss_raw_data+name);
    fetch_list()
}

function process(){
    $.ajax({
        type: 'GET',
        url: '../../process_all',
        dataType: 'json',
        success: function(data) {
        },
        async: true
    });
}

function show_status(){
    $.ajax({
        type: 'GET',
        url: '../../show_auto_proc_status',
        success: function(data) {
            document.getElementById("proc_progress").innerHTML=data
        },
        async: true
    });
}

$(document).ready(function(){
    fetch_list()
    var int=self.setInterval("upload_list()",1000);
})

