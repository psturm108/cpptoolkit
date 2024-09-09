#ifndef SECURE_STREAM_H
#define SECURE_STREAM_H

#include "types.h"


class secureContext {
  /**The secureContext stream is all the needed information to store a single client-side
   * ssl connection. This can and probably will be split into smaller pieces... eventually.
   * It works for now.
  */
  public:
    secureContext(){}
    virtual ~secureContext(){}

    virtual int begin();
    virtual int bind(int desc);
    virtual int complete();
    virtual void end();

    int descriptor;

    BIO              *certbio = NULL;
    BIO               *outbio = NULL;
    X509                *cert = NULL;
    X509_NAME       *certname = NULL;
    const SSL_METHOD *method;
    SSL_CTX *ctx;
    SSL *ssl;
  };
  inline int secureContext::bind(int desc){
    this->descriptor = desc;
    /* ---------------------------------------------------------- *
     * Attach the SSL session to the socket descriptor            *
     * ---------------------------------------------------------- */
    SSL_set_fd(this->ssl,this->descriptor);
    return 0;
  }
  inline int secureContext::complete(){


    /* ---------------------------------------------------------- *
     * Try to SSL-connect here, returns 1 for success             *
     * ---------------------------------------------------------- */
    if ( SSL_connect(this->ssl) != 1 ){
      //! Error occured
    }
      //BIO_printf(this->outbio, "Error: Could not build a SSL session to: %s.\n", dest_url);
    //else
      //BIO_printf(this->outbio, "Successfully enabled SSL/TLS session to: %s.\n", dest_url);

    /* ---------------------------------------------------------- *
     * Get the remote certificate into the X509 structure         *
     * ---------------------------------------------------------- */
    this->cert = SSL_get_peer_certificate(this->ssl);
    if (this->cert == NULL){
      //! Error occured
    }

    /* ---------------------------------------------------------- *
     * extract various certificate information                    *
     * -----------------------------------------------------------*/
    this->certname = X509_NAME_new();
    this->certname = X509_get_subject_name(this->cert);

    /* ---------------------------------------------------------- *
     * display the cert subject here                              *
     * -----------------------------------------------------------*/
    BIO_printf(this->outbio, "Displaying the certificate subject data:\n");
    X509_NAME_print_ex(this->outbio, this->certname, 0, 0);
    BIO_printf(this->outbio, "\n");

    /* ---------------------------------------------------------- *
     * Free the structures we don't need anymore                  *
     * -----------------------------------------------------------*/
     return 0;
  }
  inline int secureContext::begin(){
    this->certbio = BIO_new(BIO_s_file());
    this->outbio  = BIO_new_fp(stdout, BIO_NOCLOSE);

    if(SSL_library_init() < 0){
      BIO_printf(this->outbio, "Could not initialize the OpenSSL library !\n");
      return -1;
    }
    this->method = SSLv23_client_method();
    if ( (this->ctx = SSL_CTX_new(this->method)) == NULL){
      BIO_printf(this->outbio, "Unable to create a new SSL context structure.\n");
      return -1;
    }

    /* ---------------------------------------------------------- *
     * Disabling SSLv2 will leave v3 and TSLv1 for negotiation    *
     * ---------------------------------------------------------- */
    SSL_CTX_set_options(this->ctx, SSL_OP_NO_SSLv2);
    SSL_CTX_set_mode(this->ctx,SSL_MODE_ASYNC);

    /* ---------------------------------------------------------- *
     * Create new SSL connection state object                     *
     * ---------------------------------------------------------- */
    this->ssl = SSL_new(this->ctx);

    /* ---------------------------------------------------------- *
     * Make the underlying TCP socket connection                  *
     * ---------------------------------------------------------- */
    //server = create_socket(dest_url, outbio);
    //if(server != 0)
      //BIO_printf(outbio, "Successfully made the TCP connection to: %s.\n", dest_url);



    return 0;
  }
  inline void secureContext::end(){
    SSL_free(this->ssl);
    X509_free(this->cert);
    SSL_CTX_free(this->ctx);
    //BIO_printf(this->outbio, "Finished SSL/TLS connection with server: %s.\n", dest_url);

  }

#endif