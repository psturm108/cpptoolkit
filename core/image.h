#ifndef IMAGE_H
#define IMAGE_H

#include "types.h"
#include "vector.h"
#include "handle.h"
#include "simd.h"
#include "matrix.h"

//! The perfect place to use this bad boy
//! #include "simd.h"

#define IMAGE_OP_COPY_SOURCE                0
#define IMAGE_OP_COPY_DEST                  1

  const f32 pixelFactor = 1.0f/255.0f;
  const vec2<int> nullVector(0,0);

  //! Used for loading jpegs
  struct imgRawImage {
    unsigned int numComponents;
    unsigned long int width, height;

    unsigned char* lpData;
  };

  struct __attribute__((__packed__)) tgaHeader {
    u8 Header[12]; // File Header To Determine File Type
  };

  struct __attribute__((__packed__)) tgaInfo {
    u8 header[6];    // Holds The First 6 Useful Bytes Of The File
    u32 bytesPerPixel; // Number Of BYTES Per Pixel (3 Or 4)
    u32 imageSize;     // Amount Of Memory Needed To Hold The Image
    u32 type;          // The Type Of Image, GL_RGB Or GL_RGBA
    u32 Height;        // Height Of Image
    u32 Width;         // Width Of Image
    u32 Bpp;           // Number Of BITS Per Pixel (24 Or 32)
  };

  struct __attribute__((__packed__)) bitmapFileHeader {
    char magic[2];
    u32 size;
    u16 reserved0;
    u16 reserved1;
    u32 pixelOffset;
  };

  struct __attribute__((__packed__)) bitmapInfoHeader {
    u32 headerSize;
    s32 imageWidth;
    s32 imageHeight;
    s16 colorPlanes;
    s16 bitsPerPixel;
    u32 compression;
    u32 imageSize;
    s32 horizontalRes;
    s32 verticalRes;
    u32 colorPalette;
    u32 importantColors;
  };

  inline int roundUp(int numToRound, int multiple)
  {
      if (multiple == 0)
          return numToRound;

      int remainder = numToRound % multiple;
      if (remainder == 0)
          return numToRound;

      return numToRound + multiple - remainder;
  }


  enum imageFormatType : u8 {
    IFT_NULL=0,
    IFT_RGB,
    IFT_RGBA,
    IFT_BGR,
    IFT_HSV,
    IFT_HSL,
    IFT_GREY,
    IFT_GREY_ALPHA,
    IFT_CMYK,
    IFT_YUYV
  };

  enum imageChannel : u8 {
    IC_RED,
    IC_GREEN,
    IC_BLUE,
    IC_ALPHA
  };

  class image {
  private:
    virtual int alloc(int w,int h,int d);
    virtual void dealloc();
  public:
    //! Empty constructor
    image():
    width(0),
    height(0),
    depth(0),
    pointer(0),
    type(IFT_RGB)
    {

    }
    //! Blank (white) constructor
    image(int w,int h,int d=3,imageFormatType t=IFT_RGB):
    width(w),
    height(h),
    depth(d),
    pointer(0),
    type(t)
    {
      this->alloc(w,h,d);
    }
    //! Copy constructor
    image(const image& img){
      this->width = img.width;
      this->height = img.height;
      this->depth = img.depth;
      this->type = img.type;

      this->alloc(this->width,this->height,this->depth);
      //! This will copy the complete memory over. Happy days.
      memcpy(&this->pointer[0],&img.pointer[0], (this->width*this->height*this->depth) );
    }
    virtual ~image(){
      this->dealloc();
    }

    //! Unsafe if you don't know what your doing
    virtual image& operator=(const image& img){
      this->width = img.width;
      this->height = img.height;
      this->depth = img.depth;
      this->type = img.type;

      this->pointer = img.pointer;
      return *this;
    }

    //! Get a specific channel into a new image
    virtual image* getChannel(imageChannel ch);
    //! Seperate all channels into new images
    virtual void getChannels(image *r,image* g,image* b,image *a);
    //! Extract a matrix from a specific colour channel
    template<class T>
    matrix<T>* extract(const vec2<int>& tl,const vec2<int>& br,matrix<T> **mat,imageChannel channel);
    //! Block level image transfer
    virtual void blit(const u8* buffer,const vec2<int>& source,const vec2<int>& translation,const vec2<int>& scale,int dp=3,int mode=0);
    //! Block level image transfer
    virtual void blit(image *other,const vec2<int>& translation,const vec2<int>& scale,int mode){
      if(!other) return;
      this->blit(other->pointer,vec2<int>(other->width,other->height),translation,scale,other->depth,mode);
    }
    //! Fill the target area with values
    virtual void fill(const vec2<int>& tl,const vec2<int>& br,const rgba8& col);
    //! Draw a line from point a to point be
    virtual void drawLine(const vec2<int>& a,const vec2<int>& b,const rgba8& col=rgba8(0x00,0x00,0x00,0xFF),f32 thickness=1.0f);
    //! Get the given pixel safely
    virtual u8* sget(int x,int y);
    //! Get the given pixel unsafely. It is upto user to ensure
    //! coordinates are in range
    virtual u8* get(int x,int y);
    //! Same as get but with vector input
    virtual u8* get(const vec2<int>& point);
    //! Interpolate the given point by index
    virtual int interpolate(int x,int y);
    //! Get the given pixel(s) as a vec3<u8> value
    virtual vec3<u8> getPixel(int x,int y);
    virtual rgba8 fgetPixel(int x,int y,const rgba8& boundary);
    virtual rgba8 getPixel(const vec2<int>& point,const rgba8& boundary=rgba8(0x00,0x00,0x00,0x00));
    virtual void setPixel(const vec2<int>& point,const rgba8& color=rgba8(0x00,0x00,0x00,0x00));
    virtual void setPixel(int x,int y,const vec3<u8>& px);
    //! Works for simple RGB samples
    virtual int loadBMP(const std::string& px);
    //! Works for simple RGB samples
    virtual int saveBMP(const std::string& px);
    //! Works for simple RGB samples
    virtual int loadPNG(const std::string& px);
    //! Works for simple RGB samples
    virtual int savePNG(const std::string& px);
    //! Works for simple RGB samples
    virtual int loadJPG(const std::string& px);
    //! Works for simple RGB samples
    virtual int saveJPG(const std::string& px);
    //! Not tested / not working
    virtual int loadTGA(const std::string& px);
    //! Not tested / not working
    virtual int saveTGA(const std::string& px);
    //! Run a convolution kernel (3x3)
    template<class T>
    image* convolve3(T *kernel,image *buffered = 0);
    template<class T>
    image* convolve(const vec2<int>& dims,T *kernel,image *buffered=0);
    //! This works dope.. surprisingly
    virtual image* resize(const vec2<int>& scale);
    //! A simple alt
    virtual image* resize(float scale = 0.5f){
      vec2<int> sz(
        (int)ceil((float)this->width*scale),
        (int)ceil((float)this->height*scale)
      );

      return this->resize(sz);
    }

    /* There is a slight bug in the rotate function*/
    virtual image* rotate(float angle);

    //! Get the lowest pixel value in the set
    virtual rgba8 min();
    //! Get the highest pixel value in the set
    virtual rgba8 max();
    //! Create a greyscale copy
    virtual image* greyscale(const vec2<int>& tl,const vec2<int>& br,bool alpha);
    //! Blend two images together
    virtual void blend(image *a,image *b,u8 mode);

    //! Apply perlin noise
    //virtual void perlin(const vec2<float>& offset,const vec2<float>& bounds,int seed);

    virtual void print(){
      printf("Image: %ix%i (%i) ",this->width,this->height,this->depth);
      switch(this->type){
        case IFT_GREY: {
          printf("Greyscale\r\n");
        } break;
        case IFT_RGB: {
          printf("RGB\r\n");
        } break;
        case IFT_RGBA: {
          printf("RGBA\r\n");
        } break;
        default: {
          printf("Unknown\r\n");
        } break;
      }
    }

    u8 *pointer;

    int width;
    int height;
    int depth;

    imageFormatType type;
  };
  inline void image::getChannels(image *r,image* g,image* b,image *a){

    //! Go over each row
    for(int j=0;j<this->height;++j){
      //! Go over each col
      for(int i=0;i<this->width;++i){
        vec2<int> loc(i,j);

        rgba8 pixel = this->getPixel(loc);

        if(r){
          r->setPixel(loc,rgba8(pixel.r,0,0,0));
        }
        if(g){
          g->setPixel(loc,rgba8(pixel.g,0,0,0));
        }
        if(b){
          b->setPixel(loc,rgba8(pixel.b,0,0,0));
        }
        if(a){
          a->setPixel(loc,rgba8(pixel.a,0,0,0));
        }

      }
    }

  }
  inline image* image::getChannel(imageChannel ch){
    //! Create a new greyscale image
    image *pni = new image(this->width,this->height,1,IFT_GREY);

    //! Go over each row
    for(int j=0;j<this->height;++j){
      //! Go over each col
      for(int i=0;i<this->width;++i){
        rgba8 pixel = this->getPixel(vec2<int>(i,j));

        switch(ch){
          case IC_RED: {
            pni->setPixel(vec2<int>(i,j),rgba8(pixel.r,0,0,0));
          } break;
          case IC_GREEN: {
            pni->setPixel(vec2<int>(i,j),rgba8(pixel.g,0,0,0));
          } break;
          case IC_BLUE: {
            pni->setPixel(vec2<int>(i,j),rgba8(pixel.b,0,0,0));
          } break;
          case IC_ALPHA: {
            pni->setPixel(vec2<int>(i,j),rgba8(pixel.a,0,0,0));
          } break;
        }
      }
    }
    //! Return a pointer to the image
    return pni;
  }
  template<class T>
  inline matrix<T>* image::extract(const vec2<int>& tl,const vec2<int>& br,matrix<T>** mat,imageChannel channel){
    vec2<int> boundary =  br;

    boundary -= tl;

    matrix<T>* outmat;

    if(!mat){
      //! Matrix pointer has not been defined
      outmat = new matrix<T>(boundary.x,boundary.y);
    } else if(!*mat){
      //! Matrix pointer does not point to matrix
      outmat = new matrix<T>(boundary.x,boundary.y);
    } else {
      //! We already have a preallocated matrix
      //! Check if it is big enough
      vec2<int> matsize( (*mat)->width, (*mat)->height );

      //! We will only take the largest amount we
      //! can safely
      if(matsize < boundary){
        //! The matrix can store the entire clip, take
        //! only what we can grab
        boundary = matsize;
      }

    }
    
    for(int j=0;j<boundary.height;++j){
      for(int i=0;i<boundary.width;++i){

        rgba8 pixel = this->getPixel(vec2<int>(i,j));

        switch(channel){
          case IC_RED: {
            outmat->set(vec2<int>(i,j),pixelFactor*pixel.r);
          } break;
          case IC_GREEN: {
            outmat->set(vec2<int>(i,j),pixelFactor*pixel.g);
          } break;
          case IC_BLUE: {
            outmat->set(vec2<int>(i,j),pixelFactor*pixel.b);
          } break;
          case IC_ALPHA: {
            outmat->set(vec2<int>(i,j),pixelFactor*pixel.a);
          } break;
        }

      }
    }

    return outmat;
  }
  //! Block level image transfer
  inline void image::blit(const u8* buffer,const vec2<int>& source,const vec2<int>& translation,const vec2<int>& scale,int dp,int mode){
    //! Go over each row of the output
    for(int j=0;j<scale.y;++j){
      //!Get a pointer to the input row

      //! Calculate our indices
      int sourceIndex = ((j*source.width))*dp;
      int destinationIndex = ((translation.x)+( (j+translation.y) *this->width ))*this->depth;

      //! Get our pointers
      u8 *sptr = (u8*)&buffer[ sourceIndex ];
      u8 *dptr = &this->pointer[ destinationIndex ];

      for(int i=0;i<scale.x;++i){
        //! Determine our read point
        vec2<int> readPoint(i,j);
        //! Determine our write point
        vec2<int> writePoint = translation;

        writePoint += readPoint;

        //! Ensure the pixel is readable
        if(readPoint >= source){
          //! Cannot read from this location
          //! continue;
          //! Could probably break the loop for rest of row here
          break;
        }
        if(writePoint >= vec2<int>(this->width,this->height)){
          //! Cannot write to this location
          //! continue;
          //! Could probably break the loop for rest of row here
          break;
        }

        rgba8 sourcePixel(0x00);
        rgba8 targetPixel(0x00);
        rgba8 outputPixel(0x00);

        //! We now need to read a pixel and write it
        switch(dp){
          case 1: {
            sourcePixel.r = sptr[0];
          } break;
          case 2: {
            sourcePixel.r = sptr[0];
            sourcePixel.a = sptr[1];
          } break;
          case 3: {
            sourcePixel.r = sptr[0];
            sourcePixel.g = sptr[1];
            sourcePixel.b = sptr[2];
          } break;
          case 4: {
            sourcePixel.r = sptr[0];
            sourcePixel.g = sptr[1];
            sourcePixel.b = sptr[2];
            sourcePixel.a = sptr[3];
          } break;
        }
        //! We now need to read the source pixel
        switch(this->depth){
          case 1: {
            targetPixel.r = dptr[0];
          } break;
          case 2: {
            targetPixel.r = dptr[0];
            targetPixel.a = dptr[1];
          } break;
          case 3: {
            targetPixel.r = dptr[0];
            targetPixel.g = dptr[1];
            targetPixel.b = dptr[2];
          } break;
          case 4: {
            targetPixel.r = dptr[0];
            targetPixel.g = dptr[1];
            targetPixel.b = dptr[2];
            targetPixel.a = dptr[3];
          } break;
        }

        switch(mode){
          case IMAGE_OP_COPY_SOURCE: {
            //! No op (copy (source))
            outputPixel = sourcePixel;
          } break;
          case IMAGE_OP_COPY_DEST: {
            //! No op (copy (dest))
            outputPixel = targetPixel;
          } break;
        }

        //! We now need to write said pixel
        switch(this->depth){
          case 1: {
            dptr[0] = outputPixel.r;
          } break;
          case 2: {
            dptr[0] = outputPixel.r;
            dptr[1] = outputPixel.a;
          } break;
          case 3: {
            dptr[0] = outputPixel.r;
            dptr[1] = outputPixel.g;
            dptr[2] = outputPixel.b;
          } break;
          case 4: {
            dptr[0] = outputPixel.r;
            dptr[1] = outputPixel.g;
            dptr[2] = outputPixel.b;
            dptr[3] = outputPixel.a;
          } break;
        }
        //! End of switch statement

        //! Advance the source pointer
        sptr = &sptr[dp];
        //! Advance the dest pointer
        dptr = &dptr[this->depth];
      }
    }
    return;       //! End of function
  }
  inline void image::fill(const vec2<int>& tl,const vec2<int>& br,const rgba8& col){
    vec2<int> br0 = br;
    vec2<int> tl0 = tl;

    vec2<int> op = br0-tl0;
    //! Go over each row (scanline)
    for(int i=0;i<op.y;++i){
      u8 *ptr = this->get(0,i);

      //! Go over each pixel to be operated on
      for(int j=0;j<(op.x*this->depth);j+=this->depth){
        ptr[j] = col.x;
        ptr[j+1] = col.y;
        ptr[j+2] = col.z;
        ptr[j+3] = (this->depth==4)?col.w:ptr[j+3];
      }
    }
  }
  inline void image::drawLine(const vec2<int>& a,const vec2<int>& b,const rgba8& col,f32 thickness){
    //! Copy the start vector as a float
    vec2<float> A(1.0f*a.x ,1.0f* a.y);
    //! Copy the end vector as a float
    vec2<float> B(1.0f*b.x, 1.0f* b.y);
    //! Distance
    vec2<float> D = B-A;

    //! Get absolute distance
    D.abs();

    //! Determine line gradient
    float g = D.y / D.x;

    //! Center the two vectors
    A+=0.5f;
    B+=0.5f;

    //! Get the distance from a to b
    int dist = ceil(A.distance(B));
    //! Get the upper bounds on thickness
    int tceil = ceil(thickness);

    //! Go over each point on the line
    for(int i = 0;i<dist;i++){
      //! We have now advanced 1.0f along our line

      //! We need to determine where we are

      //! Lerp to current location.
      vec2<float> point = A.lerp(B,(1.0/dist)*i);

    }

    return;
  }
  inline u8* image::sget(int x,int y){
    if(x<0 || x>=this->width) return 0;
    if(y<0 || y>=this->height) return 0;
    u8* px = &this->pointer[ (x+(y*this->width)) * this->depth ];
    return px;
  }
  inline u8* image::get(int x,int y){
    u8* px = &this->pointer[ (x+(y*this->width)) * this->depth ];
    return px;
  }
  inline u8* image::get(const vec2<int>& point){
    u8* px = &this->pointer[ (point.x+(point.y*this->width)) * this->depth ];
    return px;
  }
  inline int image::interpolate(int x,int y){
    return (x+(y*this->width)) * this->depth;
  }
  /** This function gets the 8 neighbouring pixels
   * or 'boundary' if it doesn't exist eg, Out-of-bounds
  */
  inline vec3<u8> image::getPixel(int x,int y){
    u8* px = &this->pointer[ (x+(y*this->width)) * depth ];
    return vec3<u8>(px[0],px[1],px[2]);
  }
  inline rgba8 image::fgetPixel(int x,int y,const rgba8& boundary){
    if(x<0 || x>=this->width) return boundary;
    if(y<0 || y>=this->height) return boundary;

    int ix = x+(y*this->width);
    u8 *ptr = &this->pointer[ix*this->depth];

    rgba8 value;

    for(u8 i=0;i<this->depth;++i){
      value.values[i] = ptr[i];
    }
    return value;
  }
  inline rgba8 image::getPixel(const vec2<int>& point,const rgba8& boundary){
    rgba8 value = boundary;

    if(point.x<0 || point.x>=this->width) return value;
    if(point.y<0 || point.y>=this->height) return value;

    int index = point.x+(point.y*this->width);

    u8 *ptr = &this->pointer[index*this->depth];

    switch(this->depth){
      case 1: {
        value.r = ptr[0];
      } break;
      case 2: {
        value.r = ptr[0];
        value.a = ptr[1];
      } break;
      case 3: {
        value.r = ptr[0];
        value.g = ptr[1];
        value.b = ptr[2];
      } break;
      case 4: {
        value.r = ptr[0];
        value.g = ptr[1];
        value.b = ptr[2];
        value.a = ptr[3];
      } break;
      default: {
        //! Unsupported value
      } break;
    }
    return value;
  }
  inline void image::setPixel(const vec2<int>& point,const rgba8& color){
    if(point.x<0 || point.x>=this->width) return;
    if(point.y<0 || point.y>=this->height) return;

    u8 *ptr = &this->pointer[this->depth*(point.x+(point.y*this->width))];

    switch(this->depth){
      case 1: {
        ptr[0] = color.r;
      } break;
      case 2: {
        ptr[0] = color.r;
        ptr[1] = color.a;
      } break;
      case 3: {
        ptr[0] = color.r;
        ptr[1] = color.g;
        ptr[2] = color.b;
      } break;
      case 4: {
        ptr[0] = color.r;
        ptr[1] = color.g;
        ptr[2] = color.b;
        ptr[3] = color.a;
      } break;
    }
  }
  inline void image::setPixel(int x,int y,const vec3<u8>& px){
    u8* pxz = &this->pointer[ (x+(y*this->width)) * depth ];

    pxz[0] = px.r;
    pxz[1] = px.g;
    pxz[2] = px.b;
  }
  inline int image::alloc(int w,int h,int d){
    this->pointer = (u8*) malloc(w*h*d);
    if(!this->pointer){
      return -1;
    }
    memset(&this->pointer[0],0,w*h*d);
    this->width = w;
    this->height = h;
    this->depth = d;
    return 0;
  }
  inline void image::dealloc(){
    if(this->pointer)
      free(this->pointer);
    this->pointer=0;
    this->width = 0;
    this->height = 0;
    this->depth = 0;
  }
  inline int image::loadBMP(const std::string& px){
    //! Open the file handle
    handle ph(HT_FILE);
    //! Actually open the handle
    if(ph.open(px.c_str())==-1){
      printf("Could not open file: %s\r\n",px.c_str());
    }
    //! Return the size of the file
    s64 size = ph.size();

    //! Read in the complete file
    printf("Reading image file of size: %lld bytes\r\n",size);
    //! Create a buffer of that size
    u8 *buff = (u8*) malloc(size);

    s32 bytesRead = 0;
    do {
      bytesRead+=ph.read(&buff[bytesRead],(size-bytesRead));
    } while(bytesRead < size);

    //! Cast our memory buffer to the header pointer
    bitmapFileHeader *headerPointer = (bitmapFileHeader*)&buff[0];
    //! Cast our memory buffer (at 0x0E) to the info pointer
    bitmapInfoHeader *infoPointer = (bitmapInfoHeader*)&buff[14];

    //! Now we can access the data within, in a safe manner

    //! Set all our internal data values
    this->width = infoPointer->imageWidth;
    this->height = infoPointer->imageHeight;
    this->depth = infoPointer->bitsPerPixel/8;

    u8 *pixelStart = &buff[headerPointer->pixelOffset];

    //headerPointer->pixelOffset >>=16;
    //! Can't forget this one

    switch(this->depth){
      case 1: {
        this->type = IFT_GREY;
      } break;
      case 2: {
        this->type = IFT_GREY_ALPHA;
      } break;
      case 3: {
        this->type = IFT_RGB;
      } break;
      case 4: {
        this->type = IFT_RGBA;
      } break;
    }

    //! Allocate the internal pixel buffer
    this->alloc(this->width,this->height,this->depth);
    //! To do:
    printf("Memory allocated\r\n");
    //! Move the data over to the right place.
    printf("\t- Magic numbers: %c%c\r\n",headerPointer->magic[0],headerPointer->magic[1]);
    printf("\t- Header size: %u\r\n",infoPointer->headerSize);
    printf("\t- Bits per pixel: %i\r\n",infoPointer->bitsPerPixel);
    printf("\t- Image width: %i\r\n",infoPointer->imageWidth);
    printf("\t- Image height: %i\r\n",infoPointer->imageHeight);
    printf("\t- Image size: %i\r\n",infoPointer->imageSize);

    printf("\t- Color planes: %i\r\n",infoPointer->colorPlanes);
    printf("\t- Compression: %i\r\n",infoPointer->compression);
    printf("\t- H Resolution: %i\r\n",infoPointer->horizontalRes);
    printf("\t- V Resolution: %i\r\n",infoPointer->verticalRes);
    printf("\t- Color palette: %u\r\n",infoPointer->colorPalette);
    printf("\t- Important: %u\r\n",infoPointer->importantColors);

    printf("\t- Pixel offset: %08X\r\n",headerPointer->pixelOffset);

    printf("\t- Header size: %lu\r\n",sizeof(bitmapFileHeader));

    printf("\t- Width: %i\r\n",this->width);
    printf("\t- Height: %i\r\n",this->height);
    printf("\t- Depth: %i\r\n",this->depth);


    //! Now to unpack data from the pixel offset until the end, in order to
    //! read the bitmap correctly. This code currently only works for standard
    //! 24 bit bitmaps

    //! Round up to nearest multiple of 4
    s32 rowSize = roundUp(infoPointer->imageWidth,4);

    printf("Packed row size:\t%i\r\n",rowSize);

    //! Check if size divisible by 4
    //if(rowSize == this->width){
      //printf("Fast copy\r\n");
      //memcpy(&this->pointer[0],&pixelStart[0],this->width*this->height*this->depth);
    //} else {

    //! For each row
    for(int j=0;j<this->height;++j){
      //! Go over the row and unpack the pixels

      //! Get the inverse of j
      int antij = this->height - j -1;
      //! For each pixel
      for(int i=0;i<this->width;++i){
        //! Get the pixel offset;
        s32 dindex = i+(j*this->width);
        //! This will need to include some padding
        //! eventually
        s32 sindex = i+(antij*rowSize);

        //! Get the destination pointer
        u8 *dptr = &this->pointer[dindex*this->depth];
        //! Get the source pointer
        u8 *sptr = &pixelStart[sindex*this->depth];

        //! Translate it around because of the pixel format
        switch(this->type){
          case IFT_GREY: {
            dptr[0] = sptr[0];
          } break;
          case IFT_GREY_ALPHA: {
            dptr[0] = sptr[0];
            dptr[1] = sptr[1];
          } break;
          case IFT_BGR: {
            //! Direct copy
            dptr[0] = sptr[0];
            dptr[1] = sptr[1];
            dptr[2] = sptr[2];

          } break;
          case IFT_RGB: {
            //! Modified copy
            dptr[0] = sptr[2];
            dptr[1] = sptr[1];
            dptr[2] = sptr[0];

          } break;
        };

      }
    }

    //}

    //! By this stage, the image should have been copied over
    //! Happy days

    printf("\t- Row size: %d\r\n",rowSize);

    //! Free the memory
    free(buff);
    //! Close the file handle
    ph.close();

    return 0;
  }
  inline int image::saveBMP(const std::string& px){
    //! Open the file handle
    handle ph(HT_FILE);
    //! Actually open the handle
    if(ph.open(px.c_str(),O_WRONLY | O_CREAT)<0){
      printf("Could not open file: %s\r\n",px.c_str());
    }
    //! Round up to nearest multiple of 4
    s32 rowSize = roundUp(this->width,4);
    //! Determine the size of the output file
    s64 size = 1+sizeof(bitmapFileHeader) + sizeof(bitmapInfoHeader) + (rowSize*this->height*this->depth);

    printf("Writing image file of size: %lld bytes\r\n",size);
    //! Create a buffer of that size
    u8 *buff = (u8*) malloc(size);
    
    memset(&buff[0],0,size);

    //! Cast our memory buffer to the header pointer
    bitmapFileHeader *headerPointer = (bitmapFileHeader*)&buff[0];
    //! Cast our memory buffer (at 0x0E) to the info pointer
    bitmapInfoHeader *infoPointer = (bitmapInfoHeader*)&buff[14];

    //! Now we can access the data within, in a safe manner

    //! Set all our internal data values
    infoPointer->imageWidth = this->width;
    infoPointer->imageHeight = this->height;
    infoPointer->bitsPerPixel = this->depth*8;
    infoPointer->headerSize = 40;
    infoPointer->imageSize = rowSize*this->height*this->depth;
    infoPointer->colorPlanes = 1;
    infoPointer->importantColors = 256;
    infoPointer->colorPalette = 256;
    infoPointer->horizontalRes = 2867;
    infoPointer->verticalRes = 2867;
    headerPointer->magic[0] = 'B';
    headerPointer->magic[1] = 'M';
    headerPointer->pixelOffset = 0x37;
    headerPointer->size = size;


    //headerPointer->pixelOffset >>=16;
    //! Can't forget this one
    this->type = IFT_RGB;

    switch(this->depth){
      case 1: {
        this->type = IFT_GREY;
        //! Use run length encoding
        //infoPointer->compression = 1;
      } break;
      case 2: {
        this->type = IFT_GREY_ALPHA;
      } break;
      case 3: {
        this->type = IFT_RGB;
      } break;
      case 4: {
        this->type = IFT_RGBA;
      } break;
    }

    //! Move the data over to the right place.
    printf("\t- Magic numbers: %c%c\r\n",headerPointer->magic[0],headerPointer->magic[1]);
    printf("\t- Header size: %u\r\n",infoPointer->headerSize);
    printf("\t- Bits per pixel: %i\r\n",infoPointer->bitsPerPixel);
    printf("\t- Image width: %i\r\n",infoPointer->imageWidth);
    printf("\t- Image height: %i\r\n",infoPointer->imageHeight);
    printf("\t- Image size: %i\r\n",infoPointer->imageSize);

    printf("\t- Color planes: %i\r\n",infoPointer->colorPlanes);
    printf("\t- Compression: %i\r\n",infoPointer->compression);
    printf("\t- H Resolution: %i\r\n",infoPointer->horizontalRes);
    printf("\t- V Resolution: %i\r\n",infoPointer->verticalRes);
    printf("\t- Color palette: %u\r\n",infoPointer->colorPalette);
    printf("\t- Important: %u\r\n",infoPointer->importantColors);

    printf("\t- Pixel offset: %08X\r\n",headerPointer->pixelOffset);

    printf("\t- Header size: %lu\r\n",sizeof(bitmapFileHeader));
    printf("\t- Info size: %lu\r\n",sizeof(bitmapInfoHeader));

    //! Now to unpack data from the pixel offset until the end, in order to
    //! read the bitmap correctly. This code currently only works for standard
    //! 24 bit bitmaps

    printf("Packed row size:\t%i\r\n",rowSize);

    //! For each row
    for(int j=0;j<this->height;++j){
      //! Go over the row and unpack the pixels

      //! Get the inverse of j
      int antij = this->height - j -1;
      //! For each pixel
      for(int i=0;i<this->width;++i){
        //! Get the pixel offset;
        s32 dindex = i+(j*this->width);
        //! This will need to include some padding
        //! eventually
        s32 sindex = headerPointer->pixelOffset+ ((i+(antij*rowSize))*this->depth);

        //! Get the source pointer
        u8 *sptr = &this->pointer[dindex*this->depth];
        //! Get the destination pointer
        u8 *dptr = &buff[sindex];

        //! Translate it around because of the pixel format
        switch(this->type){
          case IFT_GREY: {
            dptr[0] = sptr[0];
          } break;
          case IFT_GREY_ALPHA: {
            dptr[0] = sptr[0];
            dptr[1] = sptr[1];
          } break;
          case IFT_BGR: {
            //! Direct copy
            dptr[0] = sptr[0];
            dptr[1] = sptr[1];
            dptr[2] = sptr[2];

          } break;
          case IFT_RGB: {
            //! Modified copy
            dptr[0] = sptr[2];
            dptr[1] = sptr[1];
            dptr[2] = sptr[0];
          } break;
          case IFT_RGBA: {
            dptr[0] = sptr[0];
            dptr[1] = sptr[1];
            dptr[2] = sptr[2];
            dptr[3] = sptr[3];
          } break;
        };

      }
    }

    //! By this stage, the image should have been copied over
    //! Happy days

    printf("\t- Row size: %d\r\n",rowSize);

    //! Write out the buffer
    s32 bytesWritten = 0;
    do {
      bytesWritten+=ph.write(&buff[bytesWritten],(size-bytesWritten));
    } while(bytesWritten < size);

    //! Close the file handle
    ph.close();
    //! Free the memory
    free(buff);
    return 0;
  }
  inline int image::loadPNG(const std::string& px){
    FILE *fp = fopen(px.c_str(),"rb");
    if(!fp){
      return -1;
    }

    char buff[8] = {0,};
    if(fread(&buff[0],1,8,fp)<0){
      delete fp;
      return -1;
    }

    //! Check png signature
    if (png_sig_cmp((png_const_bytep)&buff[0], 0, 8)){
      //! Cancel the operation
      fclose(fp);
      return -1;
    }

    png_structp png_ptr;
    png_byte color_type;
    png_byte bit_depth;

    png_infop info_ptr;
    int number_of_passes;
    png_bytep * row_pointers;

    /* initialize stuff */
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr){
      //! Could not create read struct
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr){
      //! Could not create info struct
    }

    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not initialise io
    }

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);

    png_read_info(png_ptr, info_ptr);

    this->width = png_get_image_width(png_ptr, info_ptr);
    this->height = png_get_image_height(png_ptr, info_ptr);
    color_type = png_get_color_type(png_ptr, info_ptr);
    bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    //! Get the number of bytes per row
    int bytesPerRow = png_get_rowbytes(png_ptr,info_ptr);

    printf("Loading PNG image\r\n");
    printf("\t- Width: %i\r\n",this->width);
    printf("\t- Height: %i\r\n",this->height);
    printf("\t- Type: %02x\r\n",color_type);
    printf("\t- Depth: %02x\r\n",bit_depth);
    printf("\t- Rows: %i\r\n",bytesPerRow);

    number_of_passes = png_set_interlace_handling(png_ptr);
    png_read_update_info(png_ptr, info_ptr);

    switch(color_type){
      case 0x01: {
        //! Simple Grey format. bit_depth may be 1,2,4,8,16
        this->type = IFT_GREY;
      } break;
      case 0x02: {
        //! Simple RGB format. bit_depth may be 8 or 16
        this->type = IFT_RGB;
        //! Set the specific depth
        this->depth = 3;
      } break;
      case 0x03: {
        //! Colour palette. Not supported
      } break;
      case 0x04: {
        //! Simple Grey / Alpha format. bit_depth may be 8 or 16
        this->type = IFT_GREY_ALPHA;

        this->depth = 2;
      } break;
      case 0x06: {
        //! Simple RGBA format. bit_depth may be 8 or 16
        this->type = IFT_RGBA;

        this->depth = 4;
      } break;
    }
    //! Allocate some memory
    this->alloc(this->width,this->height,this->depth);

    /* read file */
    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not read image
    }

    for(int y=0; y<this->height; y++){
      int index = (y*this->width)*this->depth;
      //! Read in the row data
      png_read_row(png_ptr, (png_bytep)&this->pointer[index], NULL);
    }

    //! Now we should be finished
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

    return 0;
  }
  inline int image::savePNG(const std::string& px){
    /* create file */
    FILE *fp = fopen(px.c_str(), "wb");
    if (!fp){
      return -1;
    }

    png_structp png_ptr;
    png_infop info_ptr;

    /* initialize stuff */
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    //! Setup row pointers
    png_bytep row_pointers[this->height] = {0};

    for(int y=0;y<this->height;++y){
      int index = (y*this->width) *this->depth;

      row_pointers[y] = &this->pointer[index];
    }

    if (!png_ptr){
      fclose(fp);
      return -1;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr){
      fclose(fp);
      return -1;
    }

    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not init png io
    }

    png_init_io(png_ptr, fp);


    /* write header */
    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not write header
    }

    int colorType = 0;

    //! Switch output type depending on # of channels
    switch(this->depth){
      case 1: {
        colorType = PNG_COLOR_TYPE_GRAY;
      } break;
      case 2: {
        colorType = PNG_COLOR_TYPE_GRAY_ALPHA;
      } break;
      case 3: {
        colorType = PNG_COLOR_TYPE_RGB;
      } break;
      case 4: {
        colorType = PNG_COLOR_TYPE_RGB_ALPHA;
      } break;
    }

    png_set_IHDR(png_ptr, info_ptr, this->width, this->height,
      8, colorType, PNG_INTERLACE_NONE,
      PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    /* write bytes */
    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not write bytes
    }
    //! Write out image
    png_write_image(png_ptr, row_pointers);


    /* end write */
    if (setjmp(png_jmpbuf(png_ptr))){
      //! Could not finalise write
    }
    png_write_end(png_ptr, NULL);

    //! Close the given file
    fclose(fp);
    return 0;
  }
  inline int image::loadJPG(const std::string& px){
    struct jpeg_decompress_struct info;
    struct jpeg_error_mgr err;

    struct imgRawImage* lpNewImage;

    unsigned long int imgWidth, imgHeight;
    int numComponents;

    unsigned long int dwBufferBytes;

    unsigned char* lpRowBuffer[1];

    FILE* fHandle;

    fHandle = fopen(px.c_str(), "rb");
    if(fHandle == NULL) {
      return -1; /* ToDo */
    }

    info.err = jpeg_std_error(&err);
    jpeg_create_decompress(&info);

    jpeg_stdio_src(&info, fHandle);
    jpeg_read_header(&info, TRUE);

    jpeg_start_decompress(&info);
    this->width = info.output_width;
    this->height = info.output_height;
    this->depth = info.num_components;
    
    switch(info.out_color_space){
      case JCS_GRAYSCALE: {
        this->type = IFT_GREY;
      } break;
      case JCS_RGB:
      default: {
        this->type = IFT_RGB;
      } break;
    }

    dwBufferBytes = this->width*this->height*this->depth; /* We only read RGB, not A */

    printf("Loading JPG image:\r\n");
    printf("\t- Width: %i\r\n",this->width);
    printf("\t- Height: %i\r\n",this->height);
    printf("\t- Depth: %i\r\n",this->depth);

    this->alloc(this->width,this->height,this->depth);
    unsigned char* lpData = this->pointer;

    lpNewImage = (struct imgRawImage*)malloc(sizeof(struct imgRawImage));
    lpNewImage->numComponents = numComponents;
    lpNewImage->width = this->width;
    lpNewImage->height = this->height;
    lpNewImage->lpData = this->pointer;

    /* Read scanline by scanline */
    while(info.output_scanline < info.output_height) {
      lpRowBuffer[0] = (unsigned char *)(&lpData[this->depth*info.output_width*info.output_scanline]);
      jpeg_read_scanlines(&info, lpRowBuffer, 1);
    }

    free(lpNewImage);
    jpeg_finish_decompress(&info);
    jpeg_destroy_decompress(&info);
    fclose(fHandle);

    return 0;
  }
  inline int image::saveJPG(const std::string& px){
    struct jpeg_compress_struct info;
    struct jpeg_error_mgr err;

    unsigned char* lpRowBuffer[1];

    FILE* fHandle;

    fHandle = fopen(px.c_str(), "wb");
    if(fHandle == NULL) {
      return -1;
    }

    info.err = jpeg_std_error(&err);
    jpeg_create_compress(&info);

    jpeg_stdio_dest(&info, fHandle);
    struct imgRawImage lpImage;

    lpImage.width = this->width;
    lpImage.height = this->height;
    lpImage.numComponents = this->depth;
    lpImage.lpData = this->pointer;

    info.image_width = this->width;
    info.image_height = this->height;
    info.input_components = this->depth;
    switch(this->type){
      case IFT_RGBA: {
        info.in_color_space = JCS_EXT_RGBA;
      } break;
      case IFT_RGB:
      default: {
        info.in_color_space = JCS_RGB;
      } break;
      case IFT_GREY: {
        info.in_color_space = JCS_GRAYSCALE;
      } break;

    }
    

    jpeg_set_defaults(&info);
    jpeg_set_quality(&info, 100, TRUE);

    jpeg_start_compress(&info, TRUE);

    /* Write every scanline ... */
    while(info.next_scanline < info.image_height) {
      lpRowBuffer[0] = &(lpImage.lpData[info.next_scanline * (lpImage.width * this->depth)]);
      jpeg_write_scanlines(&info, lpRowBuffer, 1);
    }

    jpeg_finish_compress(&info);
    fclose(fHandle);

    jpeg_destroy_compress(&info);
    return 0;
  }
  inline int image::loadTGA(const std::string& px){
    handle fh(HT_FILE);

    //! Open the file
    if(fh.open(px.c_str(),O_RDONLY) <0) return -1;

    //! Get the total size of the file
    s64 fsize = fh.size();

    //! Allocate memory for the file
    u8 *buff = (u8*) malloc(fsize);

    int bytesRead = 0;

    do {
      int inlen = fh.read(&buff[bytesRead],fsize-bytesRead);
    } while(bytesRead<fsize);

    //! Close the file now, we are done with it
    fh.close();
    //! We have read all the data in

    return 0;
  }
  inline int image::saveTGA(const std::string& px){
    return 0;
  }

  template<class T>
  inline image* image::convolve3(T *kernel,image *buffered){
    if(!kernel) return 0;
    /**Performance upgrading.
     * 
     * So far, convolution3 using parameters:
      * 
      * Width: 888
      * Height: 960
      * Depth: 3
      * 
      * 852,480 Pixels
      * 2,557,440 Bytes of RGB data
     * 
     * uses:
     * 
     *  467,830,476 (STD)
     *  467,830,343 (STD)
     * 
     *  290,014,343 (OP#1.0)
     *  259,324,153 (OP#1.1)
     * 
     *  244,834,065 (OP#1.2)
     *  241,423,179 (OP#1.3)
     *  236,304,309 (OP#1.4)
     * 
     * instructions.
     * 
     * To speed it up, we can cache previous results so we only have to do each
     * successive column. This will be known as optimistion #1.0
     * 
     * Optimisation 1.0 was a massive success. We cut out ~170,000,000 operations!
     * We can still do better though!
     * 
     * We can speed it up further by caching the calculation of the kernel * pixelFactor,
     * This will be known at optimisation #1.1.
     * 
     * Optimisation 1.1 was also a huge success, removing 30,000,000 operations!
     * That is nothing to sneeze at!
     * 
     * Optimisation 1.2 was simple. Add channel (depth) tests to the operations to
     * ensure we only compute what we need relative to the specific image. So far we
     * are down to ~90 operations per byte of data!
     * 
     * Optimisation 1.3 was easy enough. We cached the results of averageFactor * 255.0f
     * to create the constant averageFactorByByte;
     * 
     * Optimisation 1.4 rests on using integer conversion from float as a fast
     * 'ceil' function. This cut the function count by only ~6-8 (~2/byte) instructions per
     * pixel.
     * 
     * Optimisation 1.5 is the next progression of the code. The goal is to solve 4
     * to 8 pieces of the input at the exact same time in simd fashion
     * 
     * 
    */

    T combinedKernel = (kernel[0]+kernel[1]+kernel[2]+
                        kernel[3]+kernel[4]+kernel[5]+
                        kernel[6]+kernel[7]+kernel[8]);

    //! Added in OP#1.1
    //! Normalise the kernel computation
    for(int i=0;i<9;++i){
      kernel[i] *= pixelFactor;
    }

    T averageFactor = 1.0f;

    if(combinedKernel > 1.0f){
      //! Kernel has not been normalized
      averageFactor = 1.0f / combinedKernel;
    } else {
      //! Kernel has already been normalized
      averageFactor = 1.0f;
    }

    const T averageFactorByByte = averageFactor* 255.0f;

    image *img = 0;

    if(buffered ==0){
      //! Allocate memory for the new image with same properties
      img = new image(this->width,this->height,this->depth,this->type);
    } else {
      img = buffered;
    }
    //! Create a new default value
    rgba8 bounds((u8)0);

    vec2<int> midPoint(this->width/2,this->height/2);

    //! Optimisation #1.4
    //! Create a value representing the amount of 'tiles'
    int tileSize = 4;
    int tileSq=tileSize*tileSize;
    vec2<int> tileCount(this->width/tileSize,this->height/tileSize);
    vec2<int> tileRem(this->width%tileSize,this->height%tileSize);

    //! Old code:
    rgba8 pixels[9] = {
      bounds,bounds,bounds,
      bounds,this->getPixel(vec2<int>(0,0),bounds),this->getPixel(vec2<int>(1,0),bounds),
      bounds,this->getPixel(vec2<int>(0,1),bounds),this->getPixel(vec2<int>(1,1),bounds)
    };

    //! For each row
    for(int j=0;j<this->height;++j){
      //! For each pixel
      for(int i=0;i<this->width;++i){

        float red=0.0f;
        float green = 0.0f;
        float blue = 0.0f;
        float alpha = 0.0f;

        rgba8 outputPixel(0);

        int index=0;

        u8 *localPointer=0;

        if(img->depth >=1)
          red = (
            pixels[0].r*kernel[0]+
            pixels[1].r*kernel[1]+
            pixels[2].r*kernel[2]+
            pixels[3].r*kernel[3]+
            pixels[4].r*kernel[4]+
            pixels[5].r*kernel[5]+
            pixels[6].r*kernel[6]+
            pixels[7].r*kernel[7]+
            pixels[8].r*kernel[8]
          );

        if(img->depth >=2)
          green = (
            pixels[0].g*kernel[0]+
            pixels[1].g*kernel[1]+
            pixels[2].g*kernel[2]+
            pixels[3].g*kernel[3]+
            pixels[4].g*kernel[4]+
            pixels[5].g*kernel[5]+
            pixels[6].g*kernel[6]+
            pixels[7].g*kernel[7]+
            pixels[8].g*kernel[8]
          );

        if(img->depth >=3)
          blue = (
            pixels[0].b*kernel[0]+
            pixels[1].b*kernel[1]+
            pixels[2].b*kernel[2]+
            pixels[3].b*kernel[3]+
            pixels[4].b*kernel[4]+
            pixels[5].b*kernel[5]+
            pixels[6].b*kernel[6]+
            pixels[7].b*kernel[7]+
            pixels[8].b*kernel[8]
          );

        if(img->depth == 4)
          alpha = (
            pixels[0].a*kernel[0]+
            pixels[1].a*kernel[1]+
            pixels[2].a*kernel[2]+
            pixels[3].a*kernel[3]+
            pixels[4].a*kernel[4]+
            pixels[5].a*kernel[5]+
            pixels[6].a*kernel[6]+
            pixels[7].a*kernel[7]+
            pixels[8].a*kernel[8]
          );
        
        //! New code:
        outputPixel.r = (u8) (red*averageFactorByByte);
        outputPixel.g = (u8) (green*averageFactorByByte);
        outputPixel.b = (u8) (blue*averageFactorByByte);
        outputPixel.a = (u8) (alpha*averageFactorByByte);

        //! Now to write in the pixel
        index = (i+(j*img->width))*img->depth;
        //! Hello there! Thanks for reading all my comments

        //! Grab a pointer
        localPointer = &img->pointer[index];

        switch(img->depth){
          case 1: {
            localPointer[0] = outputPixel.r;
          } break;
          case 2: {
            localPointer[0] = outputPixel.r;
            localPointer[1] = outputPixel.a;
          } break;
          case 3: {
            localPointer[0] = outputPixel.r;
            localPointer[1] = outputPixel.g;
            localPointer[2] = outputPixel.b;
          } break;
          case 4: {
            localPointer[0] = outputPixel.r;
            localPointer[1] = outputPixel.g;
            localPointer[2] = outputPixel.b;
            localPointer[3] = outputPixel.a;
          } break;
          default: {
            printf("Uknown target depth\r\n");
          };
        }

        //! At the end here, we need to shift down
        //! the pixel values that we have already
        //! fetched from memory (Added OP#1.0)
        pixels[0] = pixels[1];
        pixels[3] = pixels[4];
        pixels[6] = pixels[7];

        pixels[1] = pixels[2];
        pixels[4] = pixels[5]; 
        pixels[7] = pixels[8];

        pixels[2] = this->getPixel(vec2<int>(i+1,j-1),bounds);
        pixels[5] = this->getPixel(vec2<int>(i+1,j),bounds);
        pixels[8] = this->getPixel(vec2<int>(i+1,j+1),bounds);
        
      } //! End column for loop

      //! After each row, we need to move the pixel
      //! stack 'down' by one place
      pixels[0] = bounds;
      pixels[1] = this->getPixel(vec2<int>(0,j-1),bounds);
      pixels[2] = this->getPixel(vec2<int>(1,j-1),bounds);

      pixels[3] = bounds;
      pixels[4] = this->getPixel(vec2<int>(0,j),bounds);
      pixels[5] = this->getPixel(vec2<int>(1,j),bounds);

      pixels[6] = bounds;
      pixels[7] = this->getPixel(vec2<int>(0,j+1),bounds);
      pixels[8] = this->getPixel(vec2<int>(1,j+1),bounds);

    } //! End row for loop

    //! Return the new image
    return img;
  }

template<class T>
inline image* image::convolve(const vec2<int>& dims,T *kernel,image *buffered){
  image *img;

  T combinedKernel = 0.0f;

  for(int i=0;i<dims.x;++i){
    for(int j=0;j<dims.y;++j){
      combinedKernel += kernel[i+(j*dims.x)];
    }
  }

  //! Added in OP#1.1
  //! Normalise the kernel computation
  for(int i=0;i<(dims.x*dims.y);++i){
    kernel[i] *= pixelFactor;
  }

  T averageFactor = 1.0f;

  if(combinedKernel > 1.0f){
    //! Kernel has not been normalized
    averageFactor = 1.0f / combinedKernel;
  } else {
    //! Kernel has already been normalized
    averageFactor = 1.0f;
  }

  const T averageFactorByByte = averageFactor* 255.0f;

  if(buffered ==0){
    img = new image(this->width,this->height,this->depth,this->type);
  } else {
    img = buffered;
  }

  rgba8 bounds((u8)0);

  //! Go over each row
  for(int j=0;j<this->height;++j){
    //! Go over each col
    for(int i=0;i<this->width;++i){

      vec2<int> limit = dims;

      limit -= 1;
      limit /= 2;

      rgba8 pixels[dims.x][dims.y];

      rgba8 outputPixel(0);

      float red=0;
      float green=0;
      float blue=0;
      float alpha=0;

      //! Gather all our pixel values
      for(int k=0;k<dims.x;++k){
          for(int l=0;l<dims.y;++l){

            pixels[k][l] = this->getPixel(
              vec2<int>(i-limit.x,j-limit.y),
              bounds
            );
          }
      }
      //! Multiply our values
      for(int k=0;k<dims.x;++k){
        for(int l=0;l<dims.y;++l){

          int kindex = (k+(l*dims.x));

          red   += pixels[k][l].r*kernel[kindex];
          green += pixels[k][l].r*kernel[kindex];
          blue  += pixels[k][l].r*kernel[kindex];
          alpha += pixels[k][l].r*kernel[kindex];

        }
      }

      outputPixel.r = (u8) (red*averageFactorByByte);
      outputPixel.g = (u8) (green*averageFactorByByte);
      outputPixel.b = (u8) (blue*averageFactorByByte);
      outputPixel.a = (u8) (alpha*averageFactorByByte);


      //! Scatter our pixel value
      img->setPixel(vec2<int>(i,j),outputPixel);
    }
  }

  return img;
}

  inline image* image::resize(const vec2<int>& scale){
    //! We are now going to create a new image, by resizing this one

    //! Create the new image in memory

    //! Create a ratio object
    vec2<float> ratio(
      _max<f32>(this->width,scale.width) / _min<f32>(this->width,scale.width),
      _max<f32>(this->height,scale.height) / _min<f32>(this->height,scale.height)
    );

    vec2<float> invRatio(
      (float)this->width / (float)scale.width,
      (float)this->height / (float)scale.height
    );

    //! Ceil the ratio up
    ratio.ceil();

    //ratio.print();
    //invRatio.print();


    //! Cast the ratio to integer
    vec2<int> sampleSize( (int)ratio.x,(int)ratio.y );

    vec2<int> hsampleSize = vec2<int>((int)ceil(sampleSize.x * 0.5f),(int)ceil(sampleSize.y * 0.5f));
    
    //! Allocate a new image for us. Some memory. To rememebr stuff. Remember? Remember.
    image *pni = new image(scale.x,scale.y,this->depth,this->type);

    //printf("Resizing image:\r\n%ix%i (imageSize):\r\n%ix%i (sampleSize)\r\n",scale.x,scale.y,sampleSize.x,sampleSize.y);

    //! For each row
    for(int j=0;j<scale.y;++j){

      //! For each column
      for(int i=0;i<scale.x;++i){
        //! A variable
        rgba32 averagePixel(0.0f);

        //! First, gather the average pixel from the box
        //! For each row
        for(int y = 0;y<sampleSize.y;++y){
          //! For each col
          for(int x = 0;x<sampleSize.x;++x){

            float weight = 1.0f/ (sampleSize.x*sampleSize.y);

            //! Determine the specific translation (Old code)
            //vec2<int> trans = vec2<int>(i*sampleSize.x,j*sampleSize.y)+(vec2<int>(x,y) - hsampleSize);
            //! Determine the specific translation (New code)
            //vec2<int> trans = ( vec2<int>(i,j) + vec2<int>(x,y) - hsampleSize ) * vec2<int>((int)2.0f/invRatio.x, (int)2.0f/invRatio.y);

            //! This one works well for now
            vec2<int> trans = vec2<int>((int)(i*invRatio.x),(int)(j*invRatio.y))+(vec2<int>(x,y) - hsampleSize);

            //! Load up the samples
            rgba8 pixel = this->getPixel(trans,rgba8(0));
            //! Set the specific sample
            rgba32 sample = rgba32(pixel.r*1.0f,pixel.g*1.0f,pixel.b*1.0f,pixel.a*1.0f);
            //! Factor it by its importance
            sample *= weight;
            //! Add it to the average pixel
            averagePixel += sample;

          }
        }
        //! We have now loaded our pixel and applied the weights


        int outputIndex = (i+(j*pni->width))*pni->depth;
        u8* outputPointer = &pni->pointer[outputIndex];
        switch(pni->depth){
          case 1: {
            outputPointer[0] = (u8) averagePixel.r;
          } break;
          case 2: {
            outputPointer[0] = (u8) averagePixel.r;
            outputPointer[1] = (u8) averagePixel.a;
          } break;
          case 3: {
            outputPointer[0] = (u8) averagePixel.r;
            outputPointer[1] = (u8) averagePixel.g;
            outputPointer[2] = (u8) averagePixel.b;
          } break;
          case 4: {
            outputPointer[0] = (u8) averagePixel.r;
            outputPointer[1] = (u8) averagePixel.g;
            outputPointer[2] = (u8) averagePixel.b;
            outputPointer[3] = (u8) averagePixel.a;
          } break;
        }
        //! We have set the output pixel. Hoorah!


      }
    }

    return pni;
  }
  inline image* image::rotate(float angle){
    //! Now for the almighty rotate function

    //! Get the input image size
    vec2<f32> inputImageSize(1.0f*this->width,1.0f*this->height);
    //! Divide the input size by half
    vec2<f32> midPoint = inputImageSize * 0.5f;
    //! We now have the mid point
    vec2<f32> rotationPoint(1.0f,1.0f);
    //! Create a rotation bias for the given angle
    rotationPoint.rotate(vec2<f32>(0.0f,0.0f),angle);
    //! Calculate the new point for top right corner
    vec2<f32> cornerPoints[4] = {
      (
        -rotationPoint.x*midPoint.x,
        rotationPoint.y*midPoint.y
      ),(
        rotationPoint.x*midPoint.x,
        rotationPoint.y*midPoint.y
      ),(
        rotationPoint.x*midPoint.x,
        -rotationPoint.y*midPoint.y
      ),(
        -rotationPoint.x*midPoint.x,
        -rotationPoint.y*midPoint.y
      )
    };

    //! Get the new minimum / maximum for the image size
    vec2<f32> minimum;
    
    //! Calculate the image boundary minimum
    minimum.min(
      vec2<f32>(0).min(cornerPoints[0],cornerPoints[1]),
      vec2<f32>(0).min(cornerPoints[2],cornerPoints[3])
    );

    //! Calculate the image boundary maximum
    vec2<f32> maximum;
    
    maximum.max(
      vec2<f32>(0).max(cornerPoints[0],cornerPoints[1]),
      vec2<f32>(0).max(cornerPoints[2],cornerPoints[3])
    );

    //! Calculate the output image size
    vec2<f32> outputImageSize = maximum - minimum;

    //! outputImageSize += vec2<f32>(this->width*1.0f,this->height*1.0f);

    //! Round the output image size downwards
    outputImageSize.floor();
    //! Increment the size by one
    outputImageSize += 1.0f;

    printf("Output image size:\r\n");
    printf("\t- Width: %i\r\n",(int)outputImageSize.x);
    printf("\t- Height: %i\r\n",(int)outputImageSize.y);

    //! Allocate the new image size
    image *outputImage = new image(
      (s32)outputImageSize.width,
      (s32)outputImageSize.height,
      this->depth,
      this->type
    );

    //! Now we need to loop over the original image and calculate
    //! the point for the new pixel. Each calculated point is valid
    //! for 4 pixels so there is some room for optimisation but
    //! first, we need it to work. (09.07.2024)

    //! For half of the image height
    for(int j=0;j< (this->height/2)+1;++j ){
      //! For half of the image width
      for(int i=0;i< (this->width/2)+1;++i){
        //! Calculate the source of the pixels
        //! Other quads will need inverted points
        vec2<int> inputPoint0(i,j);                             //! Top left quad
        vec2<int> inputPoint1( this->width-i,j );               //! Top right quad
        vec2<int> inputPoint2( this->width-i,this->height-j);   //! Bottom right quad
        vec2<int> inputPoint3( i,this->height-j);               //! Bottom left quad
        //! Get the source pixels
        rgba8 inputPixel0 = this->getPixel(inputPoint0);
        rgba8 inputPixel1 = this->getPixel(inputPoint1);
        rgba8 inputPixel2 = this->getPixel(inputPoint2);
        rgba8 inputPixel3 = this->getPixel(inputPoint3);

        //! Initialise the output points
        vec2<float> outputPoint0(1.0f*inputPoint0.x,1.0f*inputPoint0.y);
        vec2<float> outputPoint1(1.0f*inputPoint1.x,1.0f*inputPoint1.y);
        vec2<float> outputPoint2(1.0f*inputPoint2.x,1.0f*inputPoint2.y);
        vec2<float> outputPoint3(1.0f*inputPoint3.x,1.0f*inputPoint3.y);

        //! Calculate the rotation for output
        //outputPoint0 = midPoint + outputPoint0 * rotationPoint * vec2<f32>(-1.0f,-1.0f);
        //outputPoint1 = midPoint + outputPoint1 * rotationPoint * vec2<f32>(1.0f,-1.0f);
        //outputPoint2 = midPoint + outputPoint2 * rotationPoint * vec2<f32>(1.0f,1.0f);
        //outputPoint3 = midPoint + outputPoint3 * rotationPoint * vec2<f32>(-1.0f,1.0f);
        
        vec2<f32> center(0.0f,0.0f);

        outputPoint0.rotate(center,angle);
        outputPoint1.rotate(center,angle);
        outputPoint2.rotate(center,angle);
        outputPoint3.rotate(center,angle);

        outputPoint0.x += midPoint.x*0.5f;
        outputPoint1.x += midPoint.x*0.5f;
        outputPoint2.x += midPoint.x*0.5f;
        outputPoint3.x += midPoint.x*0.5f;
        
        //! Round the output pixels
        outputPoint0.round();
        outputPoint1.round();
        outputPoint2.round();
        outputPoint3.round();

        vec2<int> iOutputPoint0(outputPoint0.x,outputPoint0.y);
        vec2<int> iOutputPoint1(outputPoint1.x,outputPoint1.y);
        vec2<int> iOutputPoint2(outputPoint2.x,outputPoint2.y);
        vec2<int> iOutputPoint3(outputPoint3.x,outputPoint3.y);

        //printf("Point: x\t%i\ty\t%i\r\n",iOutputPoint0.x,iOutputPoint0.y);
        
        outputImage->setPixel(iOutputPoint0,inputPixel0);
        outputImage->setPixel(iOutputPoint1,inputPixel1);
        outputImage->setPixel(iOutputPoint2,inputPixel2);
        outputImage->setPixel(iOutputPoint3,inputPixel3);

      }
      
    }

    return outputImage;
  }
  //! Apply perlin noise
  /*
  inline void image::perlin(const vec2<float>& offset,const vec2<float>& bounds,int seed){
    //! Notable patterns occur where i,j are multiplied by 0.05 and octave is 4         (smooth2d)
    //! Notable patterns occur where i,j are multiplied by 0.1 and octave is 8          (smooth2d)
    
    //! Notable patterns occur where i,j are multiplied by 0.1 and persist is 1 and oct is 8      (pnoise2d)

    //! Create a noise handle
    noise::module::Perlin noiseHandle;

    //! Set the specific seed for the noise handler
    noiseHandle.SetSeed(seed);

    //noiseHandle.SetOctaveCount (6);
    //noiseHandle.SetFrequency (1.0);
    //noiseHandle.SetPersistence (0.5);

    utils::NoiseMap heightMap;
    utils::NoiseMapBuilderPlane heightMapBuilder;
    heightMapBuilder.SetSourceModule (noiseHandle);
    heightMapBuilder.SetDestNoiseMap (heightMap);
    heightMapBuilder.SetDestSize (this->width, this->height);
    heightMapBuilder.SetBounds (offset.x,bounds.x,offset.y,bounds.y);
    heightMapBuilder.Build ();

    utils::RendererImage renderer;
    utils::Image image;
    renderer.SetSourceNoiseMap (heightMap);
    renderer.SetDestImage (image);
    renderer.Render ();

    for(int j=0;j<this->height;++j){
      for(int i=0;i<this->width;++i){

        int index = i+(this->width*j);
        index *= this->depth;
        //! Now we have our index;
        utils::Color *pc = image.GetSlabPtr(i,j);
        double vx = pc->red;

        //! printf("Value: %f\r\n",vx);

        this->pointer[index] =(u8) floor(vx);
      }
    }
  }
  */
  //! Determine lowest pixel value
  inline rgba8 image::min(){
    rgba8 lowest(0xFF,0xFF,0xFF,0xFF);
    int useFast = 0;
    if(useFast == 0){
      //! Use slow version

      for (int j = 0; j < this->height; ++j){
        for(int i=0;i<this->width;++i){
          int index = (i + (j * this->width));

          u8 *pointer = &this->pointer[index*this->depth];

          rgba8 pixel(0);

          switch(this->depth){
            case 1: {
              //! Single value per pixel. Stash in alpha
              pixel.a = pointer[0];
            } break;
            case 3: {
              //! 3 (RGB) values per pixel.
              pixel.r = pointer[0];
              pixel.g = pointer[1];
              pixel.b = pointer[2];

            } break;
            case 4: {
              //! 4 (RGBA) values per pixel
              pixel.r = pointer[0];
              pixel.g = pointer[1];
              pixel.b = pointer[2];
              pixel.a = pointer[3];

            } break;
          }

          //! Maintain the lowest value
          lowest.min(lowest,pixel);

        }
      }
    } else if(useFast == 1){
      //! Use fast version

      u8 *iterator = (u8*) &this->pointer[0];

      int past = this->width*this->height*this->depth;
      u8 *lastpx = (u8*) &this->pointer[past];

      for(iterator; iterator < lastpx;iterator = &iterator[this->depth]){
        rgba8 pixel(0);

        switch (this->depth)
        {
          case 1:
          {
            //! Single value per pixel. Stash in alpha
            pixel.a = iterator[0];
          }
          break;
          case 3:
          {
            //! 3 (RGB) values per pixel.
            pixel.r = iterator[0];
            pixel.g = iterator[1];
            pixel.b = iterator[2];
          }
          break;
          case 4:
          {
            //! 4 (RGBA) values per pixel
            pixel.r = iterator[0];
            pixel.g = iterator[1];
            pixel.b = iterator[2];
            pixel.a = iterator[3];
          }
          break;
        }

        //! Maintain the lowest value
        lowest.min(lowest, pixel);
      }

    } else if (useFast == 2){
      //! Use an even faster version

      //! Load up the SIMD
      /*int past = this->width*this->height*this->depth;
      u8 *lastpx = (u8*) &this->pointer[past];

      int totalSegments = past/16;
      int remainingBytes = past%16;

      //! A workspace for us to use for shuffling
      simd128 register0(0);
      simd128 register1(0);

      simd128 lowestSpread(0);
      //! A counter, useful sometimes
      int counter= 0;
      //! Complete all the simd segments asap
      for(int i=0;i<totalSegments;++i){
        //! Get the current value into simd register
        simd128 iterator((u8 *)&this->pointer[i*16]);

        switch(this->depth){
          case 1: {

          } break;
          case 3: {
            //! This will be the hardest one to accomplish as it's not spaced well

            //! If counter overflows, return to zero
            if(counter > 2){
              //! return to zero
              counter = 0;
            }
            switch(counter){
              case 0: {
                u8 lookup0[4] = {lowest.r, lowest.g, lowest.b, lowest.r};
                u8 lookup1[4] = {lowest.g, lowest.b, lowest.r, lowest.g};
                u8 lookup2[4] = {lowest.b, lowest.r, lowest.g, lowest.b};
                u8 lookup3[4] = {lowest.r, lowest.g, lowest.b, lowest.r};

                register0.set(
                    *((u32 *)(&lookup0[0])),
                    *((u32 *)(&lookup1[0])),
                    *((u32 *)(&lookup2[0])),
                    *((u32 *)(&lookup3[0])));

                //! Get the minimum of all 8 bit values (16)
                register1.minu(register0,iterator);

                

              } break;
              case 1: {
                u8 lookup1[4] = {lowest.g, lowest.b, lowest.r, lowest.g};
                u8 lookup2[4] = {lowest.b, lowest.r, lowest.g, lowest.b};
                u8 lookup0[4] = {lowest.r, lowest.g, lowest.b, lowest.r};
                u8 lookup3[4] = {lowest.g, lowest.b, lowest.r, lowest.g};

                register0.set(
                    *((u32 *)(&lookup0[0])),
                    *((u32 *)(&lookup1[0])),
                    *((u32 *)(&lookup2[0])),
                    *((u32 *)(&lookup3[0])));
              } break;
              case 2: {
                u8 lookup2[4] = {lowest.b, lowest.r, lowest.g, lowest.b};
                u8 lookup0[4] = {lowest.r, lowest.g, lowest.b, lowest.r};
                u8 lookup1[4] = {lowest.g, lowest.b, lowest.r, lowest.g};
                u8 lookup3[4] = {lowest.b, lowest.r, lowest.g, lowest.b};

                register0.set(
                    *((u32 *)(&lookup0[0])),
                    *((u32 *)(&lookup1[0])),
                    *((u32 *)(&lookup2[0])),
                    *((u32 *)(&lookup3[0])));
              } break;
            }

            ++counter;
          } break;
          case 4: {} break;
        }
      }
      */
    }
    return lowest;
  }
  inline rgba8 image::max(){
    rgba8 highest(0x00,0x00,0x00,0x00);

    for (int j = 0; j < this->height; ++j)
    {
      for (int i = 0; i < this->width; ++i)
      {
        int index = (i + (j * this->width));

        u8 *ptr = &this->pointer[index * this->depth];

        rgba8 pixel(0);

        switch (this->depth)
        {
        case 1:
        {
            //! Single value per pixel. Stash in alpha
            pixel.a = ptr[0];
        }
        break;
        case 3:
        {
            //! 3 (RGB) values per pixel.
            pixel.r = ptr[0];
            pixel.g = ptr[1];
            pixel.b = ptr[2];
        }
        break;
        case 4:
        {
            //! 4 (RGBA) values per pixel
            pixel.r = ptr[0];
            pixel.g = ptr[1];
            pixel.b = ptr[2];
            pixel.a = ptr[3];
        }
        break;
        }

        //! Maintain the lowest value
        highest.max(highest, pixel);
      }
    }
    return highest;
  }
  inline image* image::greyscale(const vec2<int>& tl,const vec2<int>& br,bool alpha){
    vec2<int> _br = br;
    vec2<int> _tl = tl;
    vec2<int> dims(_br-_tl);

    image *newImage = new image(dims.x,dims.y,(alpha==true)?IFT_GREY_ALPHA:IFT_GREY);

    for(int j=0;j<dims.y;++j){
      for(int i=0;i<dims.x;++i){

        rgba8 pixel = this->getPixel(vec2<int>(i,j));

        float average =( (1.0f*pixel.r)+(1.0f*pixel.g)+(1.0f*pixel.b) ) / 3.0f;

        u8 value = (u8) average;

        int index = (i+(j*dims.x)) * ((alpha==false)?1:2);

        newImage->pointer[index] = value;
        if(alpha)
          newImage->pointer[index+1] = pixel.a;

      }
    }

    return newImage;
  }
inline void image::blend(image *a,image *b,u8 mode){
  
}


#endif
