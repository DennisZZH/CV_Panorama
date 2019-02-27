clear
close all

srcDir=uigetdir('Choose source directory.'); %choose the folder
allnames=dir(strcat(srcDir,'/*.JPG')); 
len=length(allnames); % # of files

ii = 1;
while (ii<=len)

name = strcat(srcDir,'/', allnames(ii).name);
name
image=imread(name); 

  if ii==1
      image1=image;
      ii=ii+1;
      name=strcat(srcDir,'/', allnames(ii).name);
      image=imread(name); 
      img=Mosaic(image,image1);
  
  else
      ii=ii+1;
      name=strcat(srcDir,'/', allnames(ii).name);
      image=imread(name);
      img=Mosaic(image,img);
 
  end
      
end

 figure
 imshow(img)


