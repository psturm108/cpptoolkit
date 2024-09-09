#ifndef URI_H
#define URI_H

#include "core.h"

namespace engine {
  class uri {
  public:
    uri(){}
    uri(const std::string& url){
      this->parse(url);
    }
    virtual ~uri(){}

    virtual uri& operator=(const uri& url){
      this->proto = url.proto;
      this->username = url.username;
      this->password = url.password;
      this->hostname = url.hostname;
      this->pathname = url.pathname;
      this->extname = url.extname;
      this->search = url.search;
      this->query = url.query;
      this->reference = url.reference;
      return *this;
    }

    virtual int parse(const std::string& url);
    virtual int parseEnd(const char *px);
    virtual void print();

    std::string proto;
    std::string username;
    std::string password;
    std::string hostname;
    std::string pathname;
    std::string extname;
    std::string search;
    std::map<std::string,std::string> query;
    std::string reference;
  };
  inline void uri::print(){
    printf("Parsed Uri:\r\n");
    printf("\tProtocol:\t\t%s\r\n",this->proto.c_str());
    printf("\tHostname:\t\t%s\r\n",this->hostname.c_str());
    printf("\tPathname:\t\t%s\r\n",this->pathname.c_str());
    printf("\tExtname:\t\t%s\r\n",this->extname.c_str());
    printf("\tSearch:\t\t\t%s\r\n",this->search.c_str());
    printf("\tReference:\t\t%s\r\n",this->reference.c_str());
  }
  inline int uri::parseEnd(const char *px){
    char *path_start = (char*) px;
    char *path_end = path_start;
    for(path_end;*path_end != '#' && *path_end !='?' && *path_end != 0x00;++path_end);

    this->pathname = std::string(path_start,(path_end-path_start));

    int lastPoint = this->pathname.find_last_of('.');

    if(lastPoint< this->pathname.length()){
      //! An extname provided
      char *extname_start = (char*) &this->pathname.c_str()[lastPoint];

      char *extname_end = extname_start;
      for(extname_end;*extname_end != 0x00 && *extname_end != '#' && *extname_end != '?';++extname_end);

      this->extname = std::string(extname_start,extname_end - extname_start);
    }

    char *iterator = path_end;
    if(*iterator == '?'){
      char *query_start = iterator;
      char *query_end = query_start+1;
      for(query_end;*query_end!= '#' && *query_end != 0x00;++query_end);

      iterator = query_end;

      this->search = std::string(query_start,(query_end-query_start));
    }
    if(*iterator=='#'){
      char *reference_start = iterator;
      char *reference_end = reference_start+1;
      for(reference_end;*reference_end !=0x00;++reference_end);

      iterator = reference_end;

      this->reference = std::string(reference_start,(reference_end-reference_start));
    }
    return 0;
  }
  //! The path parser is quite large so... yeah.
  inline int uri::parse(const std::string& url){
    char *c = (char*) url.c_str();

    //! First stage is to parse the protocol spec
    char *proto_start = c;
    char *proto_end = proto_start;

    //! Scroll forward until either (null,':' or '/')
    for(proto_end;*proto_end!=':' && *proto_end!=0x00 && (*proto_end !='/' && *(proto_end+1) != '/');++proto_end);

    //! Check which value occured
    switch(*proto_end){
      case '\0': {
        //! Found null terminator first

        //! This will probably be a fatal error
        return -1;

      } break;
      case ':': {
        //! Found protocol endpoint first
        this->proto = std::string(proto_start,(proto_end-proto_start)+1);
      } break;
      case '/': {
        //! Found pathname indicator first

        //! Should probably skip forward to
        //! pathname parser

        //! Return -1 for now
        return this->parseEnd(proto_start);

      } break;
    }

    //! Second stage is to parse the hostname spec
    char *host_start = proto_end+3;
    char *host_end = host_start;
    //! Scroll forward until either the end, the port or the start of a pathname
    for(host_end;*host_end !='/' && *host_end != ':' && *host_end !=0x00;++host_end);

    switch(*host_end){
      case '\0': {
        return -1;
      } break;
      case ':': {
        return -1;
      } break;
      case '/': {
        //! Sweet, now we can move on to the path parser
        this->hostname = std::string(host_start,(host_end-host_start));
      } break;
    }

    return this->parseEnd(host_end);
  }

}

#endif
