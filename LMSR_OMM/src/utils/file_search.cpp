#include <cstring>
#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include <sys/stat.h>
 
 
// const char* filePath = "./imgs";
void GetFileNames(std::string path,std::vector<std::string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    struct stat s;
    if(!(pDir = opendir(path.c_str()))){
        std::cout<<"Folder doesn't Exist!"<<std::endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            std::string subpath=path + "/" + ptr->d_name;
            if(stat(subpath.c_str(),&s)==0)
            {
                if(s.st_mode & S_IFDIR){
                    GetFileNames(subpath,filenames);
                }else if (s.st_mode & S_IFREG){
                    filenames.push_back(path + "/" + ptr->d_name);
                }else{
                    std::cout<<"not file not directory"<<std::endl;
                }
            }
            
    }
    }
    closedir(pDir);
}

void GetFileNames(std::string path,std::string sub_name,std::vector<std::string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    struct stat s;
    if(!(pDir = opendir(path.c_str()))){
        std::cout<<"Folder doesn't Exist!"<<std::endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            std::string subpath=path + ptr->d_name;
            if(stat(subpath.c_str(),&s)==0)
            {
                if(s.st_mode & S_IFDIR){
                    GetFileNames(subpath,sub_name,filenames);
                }else if (s.st_mode & S_IFREG){
                    if(subpath.find(sub_name)!= std::string::npos &&subpath.find(".json")!= std::string::npos)
                    {
                        filenames.push_back(subpath);

                    }
                }else{
                    std::cout<<"not file not directory"<<std::endl;
                }
            }
            
    }
    }
    closedir(pDir);
}