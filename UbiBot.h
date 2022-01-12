#ifndef UBI_BOT_CAM
#define UBI_BOT_CAM
// Library for  UbiBot Livestock Monitoring
// https://www.ubibot.io/category/platform-api/
//
//    Version : @(#) 1.0
//    Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//
//  UbiHTTP
//
// Required parameters
// Description
// Name
// Enter a unique name for your UbiHTTP request.
// API Key
// API key generated automatically for UbiHTTP requests.
// Url
// Enter the web site address where the data is requested or written, starting with http:// or https://.
// Method
// Select one of the following HTTP request methods to access the web site url: GET,POST,PUT,DELETE.
// Additional parameters can be specified depending on the nature of your request. For example, UbiHTTP requests to servers that require authentication require a user name and password.
// Optional parameters
// Description
// HTTP authorized user name
// If your URL requires authentication, enter an authentication user name to access a private channel or website.
// HTTP authentication password
// If your URL needs authentication, enter the authentication password to access the private channel or website.
// Content type
// Enter the MIME or form type of the requested content. For example, application/x-www-form-ubibot.
// Host
// If your ThingHTTP request requires a host address, enter the domain name. For example, api.ubibot.cn.
// Content
// Enter the message you want to include in the request.
//

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 // pack the structures so as not to waste memory
  #define UBIPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define UBIPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define UBOT_LIB extern
#define UBI_MAX_MSG 100u                                                        // max message length in charactures

// Overview
// The Get Channel Feed Summaries API is used to read feed summaries from all the sensor fields in a channel. 
// This method can return either a JSON or CSV object. Each feed summary record consists hourly sum total, 
// average, number of records, standard deviation, maximum and minimum values of each individual field.

// doing the get
//    curl_setopt( $ch, CURLOPT_HTTP_VERSION , CURL_HTTP_VERSION_1_1 );
//    curl_setopt( $ch, CURLOPT_USERAGENT , 'JuheData' );
//    curl_setopt( $ch, CURLOPT_CONNECTTIMEOUT , 60 );
//    curl_setopt( $ch, CURLOPT_TIMEOUT , 60);
//    curl_setopt( $ch, CURLOPT_RETURNTRANSFER , true );
//    curl_setopt($ch, CURLOPT_FOLLOWLOCATION, true);
//    if( $ispost )
//    {
//        curl_setopt( $ch , CURLOPT_POST , true );
//        curl_setopt( $ch , CURLOPT_POSTFIELDS , $params );
//        curl_setopt( $ch , CURLOPT_URL , $url );
//    }
//    else
//    {
//        if($params)
//        {
//            curl_setopt( $ch , CURLOPT_URL , $url.'?'.$params );
//        }else{
//            curl_setopt( $ch , CURLOPT_URL , $url);
//        }
//    }

typedef union {
  uint16_t ubUint;                                                              // 16 bit unsigned
  int16_t ubSint;                                                               // 16 bit signed
  float32_t ubFloat;                                                            // 32 bit IEEE float
  unsigned char ubStr[64u];                                                     // text selection
  uint32_t ubUlong;                                                             // 32 bit unsigned
  int32_t ubSlong;                                                              // 32 bit signed
  uint8_t ubBool;                                                               // boolean or 8 bit byte type
} uBiValType_u;                                                                 // Union of types that may be returned as SettingType

UBIPACKED (
typedef struct
{
  int8_t keep_alive:1u;                                                         // valid message returned
  int8_t spare:7u;
  unsigned char buf[UBI_MAX_MSG];                                               // message received on interupt
  uBiValType_u value;                                                           // union of possible returned values
}) uBiConnect_t;                                                                // Union containing the reply message parsed values and status

// Where CHANNEL_ID is the ID of the target channel HTTP Method  (GET)
//
#define UBIBOT_URL_JSON_FEED(url,chan_id,parameters) { sprintf(url,"https:\/\/api.ubibot.io\/channels\/%s\/summary.json?%s",chan_id,parameters); }   // To return a JSON object
#define UBIBOT_MAX_RESULTS 8000U                                                // max number of records to retrieve

// url parameters
// php example
// $params = array(
//                "api_key" => $apikey,                                         // (string) is Read or Write key for this specific channel (no key required for public channels)
//                 "results" => 30,                                             //(integer) Number of entries to retrieve, 8000 max (optional)
//                 "start" => "",                                               //(datetime) Start date in format YYYY-MM-DD%20HH:NN:SS (optional)
//                 "end" => "",                                                 //(datetime) End date in format YYYY-MM-DD%20HH:NN:SS (optional)
//                 "timezone" => "",                                            //(string) Identifier from Time Zones Reference for this request (optional)
//                 "callback" => "",                                            //(string) Function name to be used for JSONP cross-domain requests (optional)
#define UBIBOT_CREATE_DATE_STRING(A,Yr,Mon,Day,Hr,Min,Sec) { sprintf(A,"%u-%02u-%02u\%20%02u:%02u:%02u",Yr,Mon,Day,Hr,Min,Sec); }

UBIPACKED(
typedef struct {
      unsigned char api_key[30U];                                               // Specify the API write key or read key of the channel, or token_id obtained after the user logged in.
      uint16_t results;                                                         // number of results to retrieve
      unsigned char start[25U];                                                 // start date format YYYY-MM-DD%20HH:NN:SS. use UBIBOT_CREATE_DATE_STRING to create
      unsigned char end[25U];                                                   // end date format YYYY-MM-DD%20HH:NN:SS.
      unsigned char timezone[25U];                                              // Identifier from Time Zones Reference for this request.
      unsigned char callback[50U];                                              // Function name to be used for JSONP cross-domain requests.
}) UBIBOT_Param_t;                                                              // structure holding options for url parameters

// http return codes
#define UBIBOT_HTTP_RET_CODE 200U                                               // success code
#define UBIBOT_HTTP_400ERR(code) { do{if((code>=400u) && (code<=400u)) { return 1u; } else { return 0u; }}while(0) }
#define UBIBOT_HTTP_500ERR(code) { do{if((code>=500u) && (code<=500u)) { return 1u; } else { return 0u; }}while(0) }

// html header
#define UBIBOT_HTTP_HEADER_CONT  "Content-type:text/html;charset=utf-8"         // header content

// error codes returned in ajax reply
// e.g. {"result":"error","server_time":"2017-10-09T08:53:18Z","errorCode":"permission_denied_force_log_off","desp":"account_key, or token_id is not correct"}
#define ubERRORS UB("permission_denied_force_log_off")UB("missing_data")UB("invalid_format")UB("over_limit")UB("error_method_invalid")UB("invalid_created_at")UB("invalid_json_format")UB("invalid_channel_id")UB("invalid_api_key")UB("invalid_field_value")UB("invalid_read_key")UB("invalid_timezone")UB("missing_field_data")UB("request_too_fast")UB("low_balance")UB("field_length_over_limit")UB("group_name_exist")UB("openid_not_binded")UB("account_require_verify")UB("wrong_password")
#define UB(x) #x,
const char * const UBIBOT_error_codes[] = { ubERRORS };                         // Array of error codes for json string parsing

// reply format {\"status\":0,\"code\": 0x00}
UBIPACKED(
typedef struct {
   uint16_t uBiCode;                                                            // code returned
   uint16_t uBiStatus;                                                          // status returned
}) UBIBOT_stat_t;

// This API call is used to generate a read-only key for a specified user on this channel.
#define UBIBOT_URL_JSON_ROKEY(url,chan_id,parameters) { sprintf(url,"https:\/\/api.ubibot.io\/channels\/%s\/api_keys?action=generate_read_key&%s",chan_id,parameters); }
//  Examples
//  POST http://api.ubibot.io/channels/CHANNEL_ID/api_keys?action=generate_read_key&account_key=xxxxxx-xxxxxx-xxxxxx-xxxxx
//  {“result”:”success”,”server_time”:”2017-09-04T08:59:30Z”,”read_key”:”9b11Xxxx5XbacbXa0e8dd53?}
//
// valid parameters 
// [account_key or token_id] Specify the account_key from the user account or token_id obtained after login.

// This API call is used to delete a specified read-only key for a channel.
#define UBIBOT_URL_JSON_DELKEY(url,chan_id,parameters) { sprintf(url,"https:\/\/api.ubibot.io\/channels\/%s\/api_keys?action=delete_read_key&%s",chan_id,parameters); }
// valid parameters 
// [account_key or token_id] Specify the account_key from the user account or token_id obtained after login.
// [ read_key ] the key you wish to delete


// The Import Feeds call is used to import data to a channel from a CSV file. The imported CSV file must be in the correct format.
#define UBIBOT_URL_IN_CSV(url,filnam) { sprintf(url,"https:\/\/api.ubibot.io\/%s.csv?parameters",filnam); }

// Data Forwarding Message Format
// Data forwarding is achieved by sending POST requests to the given endpoint URL. The Content-Type header is set to “application/json”. 
// The original feed data is contained in the request body as follows:
// The feed data will be forwarded in JSON format to the given URL
// The unique device channel_id: (string) can be used to distinguish the data from different devices.
// The feeds are an array consisting of:
// created_at:  the time when the data feed was sampled in ISO 8901 format.
// field1..field10: sensor readings
// status:  status information, such as SSID and ICCID
// Note that you will need to send a response to the UbiBot platform within 15 seconds for each message
// Example Forwarded Data:
// {“result”:”success”,”server_time”:”2017-09-04T06:53:34Z”,”timezone”:null,”channel”:{"channel_id":"735",
// "feeds":[{"created_at":"2018-01-05T05:51:52Z","field3":2293.119873},{"created_at":"2018-01-05T05:51:52Z",
// "field1":18.118561},{"created_at":"2018-01-05T05:51:52Z","field2":20},{"created_at":"2018-01-05T05:56:52Z",
// "field3":2180.479980},{"created_at":"2018-01-05T05:56:52Z","field1":18.382927},{"created_at":"2018-01-05T05:56:52Z",
// "field2":20},{"created_at":"2018-01-05T06:01:52Z","field3":2117.760010},{"created_at":"2018-01-05T06:01:52Z",
// "field1":18.738079},{"created_at":"2018-01-05T06:01:52Z","field2":20},{"created_at":"2018-01-05T06:02:09Z","field5":-62}]}
//
//  Requirements for Endpoint Response
// Please ensure the endpoint’s response is made within 15 seconds, otherwise the platform will close the connection.
// If the endpoint sends a response with the string “SUCCESS”, the platform will mark this forward request as successful. 
// If the endpoint sends the string “ERROR” it means the request was unsuccessful. The UbiBot platform keeps track of all the response results 
// for statistics purposes
// example
//                self.protocal_version = 'HTTP/1.1'
//                self.send_response(200)
//                self.send_header("Welcome", "Contect")
//                self.end_headers()
//                self.wfile.write(bytes("SUCCESS", "utf-8")) or self.wfile.write(bytes("ERROR", "utf-8"))
#define UBIBOT_DFMF_OK "SUCCESS"
#define UBIBOT_DFMF_FAIL "ERROR"
#define UBIBOT_DFMF_PORT 8080U

// -------------------- Example --------------------------------------------------------------------------------
//
// GET https://api.ubibot.io/channels/123/summary?api_key=XXXXXXXXXXXXX
//
// JSON Obj -> The response is a JSON object, for example:
// {"result":"success","server_time":"2019-02-07T13:13:15Z","is_truncated":false,"start":"2019-02-07T02:00:00+00:00","end":
// "2019-02-07T11:00:00+00:00","timezone":"Europe/London","num_records":10,"results":10,"channel":{"channel_id":"1419","name":"C-1419","field1"
// :"Temperature","field2":"Humidity","field3":"Light","field4":"Voltage","field5":"WIFI RSSI","field6":"Vibration Index","field7":"Knocks","field8":
// "External Temperature Probe","field9":"Reed Sensor","field10":null,"latitude":"41.7922","longitude":"123.4328","elevation":null,"created_at":"2018-12-07T03:15:40Z",
// "public_flag":"false","user_id":"8D5F3ACB-87A5-4D80-AA5F-FC64E8647990","last_entry_date":"2019-02-07T13:10:26Z","last_entry_id":"50982","vconfig":"{\"field1\":{\"h\":\"0\",\"u\":\"1\"},
// \"field2\":{\"h\":\"0\",\"u\":\"3\"},\"field3\":{\"h\":\"0\",\"u\":\"4\"},\"field4\":{\"h\":\"0\",\"u\":\"5\"},\"field5\":{\"h\":\"0\",\"u\":\"6\"},\"field6\":{\"h\":\"0\",\"u\":\"7\"},
// \"field7\":{\"h\":\"0\",\"u\":\"8\"},\"field8\":{\"h\":\"0\",\"u\":\"1\"},\"field9\":{\"h\":\"0\",\"u\":\"9\"}}","full_dump":"0","plan_code":"ubibot_free",
// "username":"cloudleader"},"feeds":[{"created_at":"2019-02-07T11:00:00+00:00","field3":{"sum":0.24,"avg":0.06,"count":4,"sd":0,"max":0.06,"min":0.06},"field1":{"sum":94.515136,"avg":23.628784,
// "count":4,"sd":0.018257971122225,"max":23.646141,"min":23.603416},"field2":{"sum":40,"avg":10,"count":4,"sd":0,"max":10,"min":10},"field5":{"sum":-160,"avg":-40,"count":4,"sd":0,"max":-40,"min":-40}},
// {"created_at":"2019-02-07T10:00:00+00:00","field3":{"sum":0.69,"avg":0.062727272727273,"count":11,"sd":0.0044536177141512,"max":0.07,"min":0.06},"field1":{"sum":260.85257,"avg":23.71387,"count":11,
// "sd":0.035359001690453,"max":23.803696,"min":23.675514},"field2":{"sum":110,"avg":10,"count":11,"sd":0,"max":10,"min":10},"field5":{"sum":-487,"avg":-44.272727272727,"count":11,"sd":4.3294112362875,
// "max":-40,"min":-49},"field4":{"sum":4.472982,"avg":4.472982,"count":1,"sd":0,"max":4.472982,"min":4.472982}},{"created_at":"2019-02-07T09:00:00+00:00","field3":{"sum":22.48,"avg":11.24,
// "count":2,"sd":1.74,"max":12.98,"min":9.5},"field1":{"sum":48.264282,"avg":24.132141,"count":2,"sd":0.021362,"max":24.153503,"min":24.110779},"field2":{"sum":20,"avg":10,"count":2,"sd":0,"max":10,
// "min":10},"field5":{"sum":-80,"avg":-40,"count":2,"sd":0,"max":-40,"min":-40}},{"created_at":"2019-02-07T08:00:00+00:00","field3":{"sum":457.879989,"avg":38.15666575,"count":12,"sd":12.868984722494,
// "max":57.32,"min":16.779999},"field1":{"sum":294.736777,"avg":24.561398083333,"count":12,"sd":0.27719641719199,"max":25.056076,"min":24.209579},"field2":{"sum":113,"avg":9.4166666666667,"count":12,
// "sd":0.49300664859163,"max":10,"min":9},"field5":{"sum":-512,"avg":-42.666666666667,"count":12,"sd":4.0892813821284,"max":-40,"min":-51},"field4":{"sum":4.475632,"avg":4.475632,"count":1,
// "sd":0,"max":4.475632,"min":4.475632}},{"created_at":"2019-02-07T07:00:00+00:00","field3":{"sum":200.879997,"avg":100.4399985,"count":2,"sd":2.6000025,"max":103.040001,"min":97.839996},
// "field1":{"sum":56.227211,"avg":28.1136055,"count":2,"sd":0.2456705,"max":28.359276,"min":27.867935},"field2":{"sum":16,"avg":8,"count":2,"sd":0,"max":8,"min":8},
// "field5":{"sum":-90,"avg":-45,"count":2,"sd":4,"max":-41,"min":-49}},{"created_at":"2019-02-07T06:00:00+00:00","field3":{"sum":31344.398927,"avg":2612.0332439167,
// "count":12,"sd":2824.6816531297,"max":7016.959961,"min":116.199997},"field1":{"sum":378.384835,"avg":31.532069583333,"count":12,"sd":2.9701401037999,"max":35.892273,
// "min":26.377892},"field2":{"sum":87,"avg":7.25,"count":12,"sd":1.0103629710818,"max":9,"min":6},"field5":{"sum":-491,"avg":-40.916666666667,
// "count":12,"sd":0.27638539919628,"max":-40,"min":-41},"field4":{"sum":4.487029,"avg":4.487029,"count":1,"sd":0,"max":4.487029,"min":4.487029}},
// {"created_at":"2019-02-07T05:00:00+00:00","field3":{"sum":197.159996,"avg":98.579998,"count":2,"sd":1.579998,"max":100.159996,"min":97},
// "field1":{"sum":46.082627,"avg":23.0413135,"count":2,"sd":0.0146865,"max":23.056,"min":23.026627},
// "field2":{"sum":20,"avg":10,"count":2,"sd":0,"max":10,"min":10},"field5":{"sum":-89,"avg":-44.5,"count":2,"sd":4.5,"max":-40,"min":-49}},
// {"created_at":"2019-02-07T04:00:00+00:00","field3":{"sum":1133.039978,"avg":94.419998166667,"count":12,"sd":6.4416674668395,"max":115.040001,
// "min":89.68},"field1":{"sum":277.075209,"avg":23.08960075,"count":12,"sd":0.015318618498007,"max":23.114746,"min":23.069351},"field2":{"sum":120,"avg":10,
// "count":12,"sd":0,"max":10,"min":10},"field5":{"sum":-535,"avg":-44.583333333333,"count":12,"sd":5.3456888133232,"max":-40,"min":-52},"field4":{"sum":4.469537,"avg":4.469537,"count":1,
// "sd":0,"max":4.469537,"min":4.469537}},{"created_at":"2019-02-07T03:00:00+00:00","field3":{"sum":153.099998,"avg":76.549999,"count":2,"sd":0.549999,"max":77.099998,"min":76},"field1":{"sum":46.242844,"avg":23.121422,
// "count":2,"sd":0.0066760000000006,"max":23.128098,"min":23.114746},"field2":{"sum":20,"avg":10,"count":2,"sd":0,"max":10,"min":10},"field5":{"sum":-82,"avg":-41,"count":2,"sd":0,"max":-41,"min":-41}},
// {"created_at":"2019-02-07T02:00:00+00:00","field3":{"sum":1153.739984,"avg":96.144998666667,"count":12,"sd":50.714404305812,"max":256.320007,"min":73.059998},"field1":{"sum":278.933775,"avg":23.24448125,
// "count":12,"sd":0.069417701157708,"max":23.352409,"min":23.141449},"field2":{"sum":122,"avg":10.166666666667,"count":12,"sd":0.37267799624997,"max":11,"min":10},"field5":{"sum":-536,"avg":-44.666666666667,
// "count":12,"sd":7.3861732687201,"max":-40,"min":-66},"field4":{"sum":4.497895,"avg":4.497895,"count":1,"sd":0,"max":4.497895,"min":4.497895}}]}
//
// If use the device alternative
// {"command":"ReadData"} on serial or ethernet/ip
//
//  You get raw reply like {"created_at":"2015-12-17T18:16:22Z","field1":0}……
//  the reset of the config is missing as it must be done by the cloud application so it is obtained via http
//
//  The following enumerated type describes the mapping of the fields
typedef enum {
  uBi_Temperature = 1u,                                                         // field one is this
  uBi_Humidity,
  uBi_Light,
  uBi_Voltage,
  uBi_WIFI_RSSI,
  uBi_VibrationIndex,
  uBi_Knocks,
  uBi_ExtTemp,
  uBi_ReedSensor,
  uBi_SoilAbsMoist,
  uBi_MagneticSwitch,                                                           // field eleven is this
  uBi_Num_of_Fields                                                             // count of the number of fields
} uBiReadFieldData_e;                                                           // map the fields to data here

UBIPACKED (
typedef struct
{
  float32_t sum;                                                                // total sum
  float32_t avg;                                                                // average value
  uint16_t count;                                                               // count
  float32_t sd;                                                                 // standard deviation
  float32_t max;                                                                // max value
  float32_t min;                                                                // min value
}) uBiReadServerData_t;                                                         // structure containing data for one field value from a server json query

typedef union {
  float32_t deviceVal;                                                          // returned on ajax reply from a device
  uBiReadServerData_t serverVal;                                                // returned on ajax reply from a server
} uBiReadDataValues_t;                                                          // Union of types that may be returned from either a device or server read of data

//  The following universal structure can be used to store the ajax replies when parsed using jasmn or frozen
//
UBIPACKED (
typedef struct
{
  unsigned char ubiDate[30u];                                                   // date and time string from ajax reply
  uint16_t channel_id;                                                          // associated channel
  uint8_t jSonTyp;                                                              // device or server
  float32_t latitude;                                                           // latitude
  float32_t longditude;                                                         // longditude
  float32_t elevation;                                                          // elavation
  uint16_t num_records;                                                         // number of records
  unsigned char fieldsGroupName[32u];                                           // name given to the group for fields e.g. feeds
  unsigned char field1Desc[32u];                                                // description given to field1 e.g Temperature
  unsigned char field2Desc[32u];                                                // description given to field2 e.g Humidity
  unsigned char field3Desc[32u];                                                // description given to field3 e.g Light
  unsigned char field4Desc[32u];                                                // description given to field4 e.g Voltage
  unsigned char field5Desc[32u];                                                // description given to field5 e.g WIFI RSSI
  unsigned char field6Desc[32u];                                                // description given to field6 e.g Vibration Index
  unsigned char field7Desc[32u];                                                // description given to field7 e.g Knocks
  unsigned char field8Desc[32u];                                                // description given to field8 e.g External Temperature Probe
  unsigned char field9Desc[32u];                                                // description given to field9 e.g Reed Sensor
  unsigned char field10Desc[32u];                                               // description given to field10 e.g soil_absolute_moisture
  unsigned char field11Desc[32u];                                               // description given to field11 e.g magnetic_switch
  uBiReadDataValues_t temperature_celcius;                                      // temperature (in this example assumes its field1 change here and json reader function if not)
  uBiReadDataValues_t humidity;                                                 // humidity
  uBiReadDataValues_t light_lux;                                                // light lux
  uBiReadDataValues_t voltage;                                                  // voltage
  uBiReadDataValues_t vibration;                                                // vibration index
  uBiReadDataValues_t knocks;                                                   // knocks
  uBiReadDataValues_t magnetic_switch;                                          // magnetic switch
  uBiReadDataValues_t soil_absolute_moisture;                                   // soil absolute mositure
  uBiReadDataValues_t ext_temp_probe_celcuis;                                   // ext temp
  uBiReadDataValues_t reed_sensor;                                              // reed sensor
  uBiReadDataValues_t wifi_rssi;                                                // wifi rssi
}) uBiReadFieldData_t;                                                          // structure containing data for one field value

UBIPACKED (
typedef struct
{
  unsigned char ubiDate[30u];                                                   // date and time string from ajax reply
  uint16_t channel_id;                                                          // associated channel
  float32_t latitude;                                                           // latitude
  float32_t longditude;                                                         // longditude
  float32_t elevation;                                                          // elavation
  uint16_t num_records;                                                         // number of records
  uint16_t h_f1;                                                                // h for field 1
  uint16_t u_f1;                                                                // u for field 1
  uint16_t h_f2;                                                                // h for field 2
  uint16_t u_f2;                                                                // u for field 2
  uint16_t h_f3;                                                                // h for field 3
  uint16_t u_f3;                                                                // u for field 3
  uint16_t h_f4;                                                                // h for field 4
  uint16_t u_f4;                                                                // u for field 4
  uint16_t h_f5;                                                                // h for field 5
  uint16_t u_f5;                                                                // u for field 5
  uint16_t h_f6;                                                                // h for field 6
  uint16_t u_f6;                                                                // u for field 6
  uint16_t h_f7;                                                                // h for field 7
  uint16_t u_f7;                                                                // u for field 7
  uint16_t h_f8;                                                                // h for field 8
  uint16_t u_f8;                                                                // u for field 8
  uint16_t h_f9;                                                                // h for field 9
  uint16_t u_f9;                                                                // u for field 9
  uint16_t h_f10;                                                               // h for field 10
  uint16_t u_f10;                                                               // u for field 10
}) uBiReadVConfig_t;                                                            // structure containing data for vconfig field value
  
// keywords channel vconfig
// 
//  Filter channels that need to be displayed
//          $filter_fields = array('field1','field2','field3','field4','field5','field6','field7','field8','field9','field10');
//  Sensor unit
//            $unit = array('temperature_celcius','temperature_feh','humidity','light_lux','voltage','WIFI_dbm','vibration','knocks','magnetic_switch','soil_absolute_moisture');

#define UBIBOT_URL_CSV_FEED(url,chan_id,parameters) { sprintf(url,"https:\/\/api.ubibot.io\/channels\/%s\/summary.csv??%s",chan_id,parameters); } // To return a CSV file:

UBIPACKED (
typedef struct
{
  uint16_t fn_th;
  uint16_t fn_light;
  uint32_t fn_mag;
  uint16_t fn_mag_int;
  uint16_t fn_acc_tap1;
  uint16_t fn_acc_tap2;
  uint16_t fn_acc_act;
  uint16_t fn_acc_min;
  uint16_t fn_bt;
  uint32_t fn_ext;
  uint16_t fn_battery;
  uint16_t fn_dp;
  unsigned char cg_data_led[3u];                                                // on / off string
}) uBiMetaData_t;                                                               // type to store values for metadata set and readback

// device commands
#define UBI_READ_PROD "ReadProduct"                                             // Read device firmware information
#define UBI_READ_WIFI "ReadWifi"                                                // Read Wifi Configuration
#define UBI_READ_META "ReadMetaData"                                            // Read device sensor configuration (metadata)
#define UBI_READ_DATA "ReadData"                                                // Read stored sensor data from device
#define UBI_READ_ELOG "GetLastError"                                            // Read Error log
#define UBI_SCAN_WIFI "ScanWifiList"                                            // Scan Wifi Access points
#define UBI_DIAG_SENSOR "CheckSensors"                                          // Run Daignostics check on sensors
#define UBI_DEL_DATA "ClearData"                                                // Delete data

// make device request A=message buffer, B=device comand defintion as above
#define UBI_MAKE_REQUEST(A,B) { sprintf(A,"\{ \"command\" : \"%s\" } \n\r",B); }

// Set up device setMetaData with metadata in JSON format
#define UBI_DEFAULT_CONFIG_BOT(A) { sprintf(A,"\{\"command\":\"SetMetaData\",\"metadata\":\"\{\"fn_th\":60,\"fn_light\":60,\"fn_mag\":120,\"fn_mag_int\":2,\"fn_acc_tap1\":0,\"fn_acc_tap2\":0,\"fn_acc_act\":0,\"fn_acc_min\":5,\"fn_bt\":120,\"fn_ext_t\":1480041854,\"fn_battery\":120,\"fn_dp\":120,\"cg_data_led\":\"on\"\}\"\}\n\r" }

// use the metadata structure to configure the ubiBot
#define UBI_SET_CONFIG_BOT(A,MetaD) { sprintf(A,"\{\"command\":\"SetMetaData\",\"metadata\":\"\{\"fn_th\":%u,\
\"fn_light\":%u,\"fn_mag\":%u,\"fn_mag_int\":%u,\"fn_acc_tap1\":%u,\"fn_acc_tap2\":%u,\"fn_acc_act\":%u,\"fn_acc_min\":%u,\
\"fn_bt\":%u,\"fn_ext_t\":%u,\"fn_battery\":%u,\"fn_dp\":%u,\"cg_data_led\":\"%s\"\}\"\}\n\r",MetaD.fn_th, \
MetaD.fn_light,MetaD.fn_mag,MetaD.fn_mag_int,MetaD.fn_acc_tap1,MetaD.fn_acc_tap2,MetaD.fn_acc_act,MetaD.fn_acc_min, \
MetaD.fn_bt,MetaD.fn_ext_t,MetaD.fn_battery,MetaD.fn_dp,MetaD.cg_data_led); }

// change wifi configuration
typedef enum {
  uBi_wpa = 0u,                                                                 // wpa option
  uBi_wep,                                                                      // wep option
  uBi_open,                                                                     // open option
} uBiWifiConfig_e;                                                              // wifi config

UBIPACKED (
typedef struct
{
  unsigned char SSID[6u];                                                       // SSID XXXXXX config/read
  unsigned char password[12u];                                                  // password XXXXXX config
  unsigned char type[4u];                                                       // type as per type defines below config
  uint32_t rssiVal;                                                             // rssi read
}) uBiWifi_t;                                                                   // wifi structure used for config and read of signal strength

#define UBI_WIFI_CONFIG(A,SSID,passwd,type) { sprintf(a,"\{\"command\":\"SetupWifi\",\"SSID\":\"%s\",\"password\": \"%s\",\"backup_ip\":\"101.201.30.5\",\"type\":\"%s\"\}",SSID,passwd,type); }
// Backup IP: IP address to use if DNS isn't working
#define UBI_SETUP_BACKUP_WIFI(A,ssid,password,dotIP,typ) { sprintf(A,"\{\"command\":\"SetupWifi\",\"SSID\":\"%s\",\"password\": \"%s\",\"backup_ip\":\"%s\",\"type\":\"%s\"\}",ssid,password,dotIP,typ ); }
// Type: WPA, WEP or OPEN
#define UBI_WIFI_TYPE_WPA "wpa"
#define UBI_WIFI_TYPE_WEP "wep"
#define UBI_WIFI_TYPE_OPEN "open"

// Define the types of value that can be returned from the JSON query
#define uBotInt (1u<<0u)
#define uBotBool (1u<<1u)
#define uBotFlo (1u<<2u)
#define uBotULong (1u<<3u)
#define uBotStr (1u<<4u)
#define uBotConn (1u<<5u)
#define uBotSLong (1u<<6u)
#define uBotSInt (1u<<7u)

// Structure for storing information from a product read on a device
UBIPACKED (
typedef struct
{
 unsigned char ProductID[15u];                                                  // "ubibot-ws1-cn",
 unsigned char SeriesNumber[10u];                                               // :"XXXXXX",
 unsigned char Host[15u];                                                       // "api.ubibot.io",
 uint16_t CHANNEL_ID;                                                           // :"XX",
 unsigned char USER_ID[10u];                                                    // :"XXX-XXXX",
 uint16_t USAGE;                                                                // :"0%",
 unsigned char firmware[20u];                                                   // :"ws1_v1.6_17_10_18",
 unsigned char backup_ip[20u];                                                  // :"xxx.xxx.xxx.xxx"}
}) uBiProductInfo_t;                                                            // structure for contianing product info request from the device

int8_t parse_uBiBot( uBiConnect_t* conn, char* uBotCmd, uint8_t uBotReadType );

/*******************************************************************************
* Function Name: parse_uBiBot
********************************************************************************
* Summary:
*  Parse the serial command for the ubiBot device request { string: value }
*
* Slave emulation to recieve config request or a value using this simple JSON
* You Reply with a suitable ajax response using the frozen library
*
* Parameters:
*   uBiConnect_t* conn, char* uBotCmd, uint8_t uBotReadType
*
* Return:
* return if fail is -1 (0xFFu) otherwiswe type bit as below
* ==============================================================================
* Unsigned Integer 16 bit = 0x1
* Boolean 1 bit = 0x2
* Float 32 bit IEEE = 0x4
* Unsigned Long Int 32 bit = 0x8
* String = 0x10
* Connection = 0x20
* Signed Long Int 32 bit = 0x40
* Signed Int 32 bit = 0x80
* String too big for read buffer (overflow) = 0x0
*
*******************************************************************************/
UBOT_LIB int8_t parse_uBiBot( uBiConnect_t* conn, char* uBotCmd, uint8_t uBotReadType )
{
  char *p;                                                                      // create char pointer for iteration
  int8_t pos=0;                                                                 // position in string counter
  int8_t retStatus=-1;                                                          // initialise return code to error

  conn->keep_alive=!strncmp(conn->buf, "{", 1u);                                // message must start with a {
  if (!conn->keep_alive) return retStatus;                                      // no then return error
  for (p=strchr(conn->buf, '{'); p; p=strchr(p, '}'))                           // for each line  { tag : value }
  {
    p++;                                                                        // increment to next char
    if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotInt))
    {
      retStatus = uBotInt;                                                      // return i16 type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->value.ubUint=atoi(p);                                               // reads an uint16_t into union
    }
    if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotSInt))
    {
      retStatus = uBotSInt;                                                     // return s16 type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespecae
      conn->value.ubSint=atoi(p);                                               // reads an int16_t into union
    }
    else if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotBool))
    {
      retStatus = uBotBool;                                                     // return bool type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->value.ubBool=strncmp(p, "true", 4u);                                // reads bool into union
    }
    else if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotFlo))
    {
      retStatus = uBotFlo;                                                        // return float type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->value.ubFloat=atof(p);                                              // reads float32_t into union
    }
    else if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotULong))
   {
      retStatus = uBotULong;                                                        // return u32 type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->value.ubSlong=atol(p);                                              // reads int32_t into union
    }
    else if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotSLong))
    {
      retStatus = uBotSLong;                                                        // return s32 type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
      conn->value.ubUlong=atol(p);                                              // reads uint32_t into union
    }
    else if (!strncmp(p, uBotCmd, strlen(uBotCmd)) && (uBotReadType == uBotStr))
    {
      retStatus = uBotStr;                                                      // return string type
      p+=strlen(uBotCmd);                                                       // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // remove whitespace
          if (pos>=UBI_MAX_MSG) return 0u;                                      // string buffer overflow
      conn->value.ubStr[pos]=*p;                                                // reads string into union
      pos=++pos % UINT8_MAX;
    }
    else if (!strncmp(p, "Connection:", 11u))                                   // connection status message
    {
      retStatus = uBotConn;
      p+=11u;                                                                   // skip command bytes
      while (*p==' ' || *p=='\t') p++;                                          // skip whitespace and tabs
      conn->keep_alive=!strncmp(p, "keep-alive", 10u);                          // set keep alive true
    }
  }
  return retStatus;                                                             // returns a byte which is a bitmap representing the type found
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif