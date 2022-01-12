#ifndef __geoJSON_
#define __geoJSON_
//  GeoJSON is a format for encoding a variety of geographic data structures
//
// Solve complex location problems from geofencing to custom routing
//
// The GeoJSON Specification (RFC 7946)
//
// To be added TopoJSON (encoding)
//
// {
//  "type": "Feature",
//  "geometry": {
//    "type": "Point",
//    "coordinates": [125.6, 10.1]
//  },
//  "properties": {
//    "name": "Dinagat Islands"
//  }
//}

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define GEOJSONPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define GEOJSONPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define GEOJSONPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define GEOJSONPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define GEOJSONPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

typedef enum {LineString = 0u, Point = 1u, Polygon = 2u, MultiLineString = 3u, MultiPolygon = 4u, GeometryCollection = 5u, FeatureCollection = 6u, Feature = 7u } geo_json_type_e;
#define GEOJSON_IS_STRING(v) do { (((v) != NULL) && ((v)->type == LineString)) }while(0)
#define GEOJSON_IS_POINT(v) do { (((v) != NULL) && ((v)->type == Point)) }while(0)
#define GEOJSON_IS_POLYGON(v) do { (((v) != NULL) && ((v)->type == Polygon)) }while(0)
#define GEOJSON_IS_MULTILINESTR(v)  do { (((v) != NULL) && ((v)->type == MultiLineString )) }while(0)
#define GEOJSON_IS_MULTIPOLYGON(v)  do { (((v) != NULL) && ((v)->type == MultiPolygon  )) }while(0)

#define GEOJSON_GET_STRING(v) do { (GEOJSON_IS_STRING(v) ? &(v)->gjType.linestring : NULL) }while(0) /** Given a LineString return a ptr to the bare string it contains, or NULL if the value is not a string. */
#define GEOJSON_GET_POINT(v) do { (GEOJSON_IS_POINT(v) ? &(v)->gjType.point : NULL) }while(0) /** Given a point return a ptr to the bare string it contains, or NULL if the value is not a point. */
#define GEOJSON_GET_POLYGON(v) do { (GEOJSON_IS_POLYGON(v) ? &(v)->gjType.polygon : NULL) }while(0) /** Given a polygon return a ptr to the bare string it contains, or NULL if the value is not a polygon. */
#define GEOJSON_GET_MULTILINESTR(v) do { (GEOJSON_IS_MULTILINESTR(v) ? &(v)->gjType.multiline : NULL) }while(0) /** Given a multiline string return a ptr to the bare string it contains, or NULL if the value is not a multiline string. */
#define GEOJSON_GET_MULTIPOLYGON(v) do { (GEOJSON_IS_MULTIPOLYGON(v) ? &(v)->gjType.multipoly : NULL) }while(0) /** Given a multiline string return a ptr to the bare string it contains, or NULL if the value is not a multiline string. */

#if defined(D_FT900)                                                            // geoJson co-ord object
typedef struct GEOJSONPACKED {
   float32_t lat;
   float32_t lon;
   float32_t height;
} geoJson_Coord_t;
#else
GEOJSONPACKED(
typedef struct {
   float32_t lat;
   float32_t lon;
   float32_t height;
}) geoJson_Coord_t;                                                      
#endif 

/* {
    "type": "Feature",
    "geometry": {
        "type": "Point",
        "coordinates": [30.5, 50.5]
    },
    "properties": null
} */
#if defined(D_FT900)                                                            // geoJson Point object
typedef struct GEOJSONPACKED {
   geoJson_Coord_t coord;
} geoJsonPoint_t;
#else
GEOJSONPACKED(
typedef struct {
   geoJson_Coord_t coord;
}) geoJsonPoint_t;                                                     
#endif 

/* {
    "type": "LineString",
    "coordinates": [[30.5, 50.5], [30.6, 50.6]]
} */
#if defined(D_FT900)                                                            // geoJson LineString object
typedef struct GEOJSONPACKED {
   geoJson_Coord_t coord[4u];
} geoJsonLineString_t;
#else
GEOJSONPACKED(
typedef struct {
   geoJson_Coord_t coord[4u];
}) geoJsonLineString_t;                                                   
#endif 

/* {
    "type": "Polygon",
    "coordinates": [[[100, 0], [101, 0], [101, 1], [100, 1], [100, 0]]]
} */
#if defined(D_FT900)                                                            // geoJson Polygon object
typedef struct GEOJSONPACKED {
   geoJson_Coord_t coord[5u];
} geoJsonPolygon_t;
#else
GEOJSONPACKED(
typedef struct {
   geoJson_Coord_t coord[5u];
}) geoJsonPolygon_t;                                                   
#endif 

/* {
    "type": "MultiLineString",
    "coordinates": [[[30.5, 50.5], [30.6, 50.6]]]
}*/
#if defined(D_FT900)                                                            // geoJson Polygon object
typedef struct GEOJSONPACKED {
   geoJson_Coord_t coord[4u];
} geoJsonMulLineStr_t;
#else
GEOJSONPACKED(
typedef struct {
   geoJson_Coord_t coord[4u];
}) geoJsonMulLineStr_t;                                                   
#endif 

/* {
    "type": "MultiPolygon",
    "coordinates": [[[[100, 0], [101, 0], [101, 1], [100, 1], [100, 0]]]]
}*/
#if defined(D_FT900)                                                            // geoJson MultiPolygon object
typedef struct GEOJSONPACKED {
   geoJson_Coord_t coord[10u];
} geoJsonMulPolygon_t;
#else
GEOJSONPACKED(
typedef struct {
   geoJson_Coord_t coord[10u];
}) geoJsonMulPolygon_t;                                                   
#endif  

/*
{
  type: "GeometryCollection",
  geometries: [
     {
       type: "MultiPoint",
       coordinates: [
          [ -73.9580, 40.8003 ],
          [ -73.9498, 40.7968 ],
          [ -73.9737, 40.7648 ],
          [ -73.9814, 40.7681 ]
       ]
     },
     {
       type: "MultiLineString",
       coordinates: [
          [ [ -73.96943, 40.78519 ], [ -73.96082, 40.78095 ] ],
          [ [ -73.96415, 40.79229 ], [ -73.95544, 40.78854 ] ],
          [ [ -73.97162, 40.78205 ], [ -73.96374, 40.77715 ] ],
          [ [ -73.97880, 40.77247 ], [ -73.97036, 40.76811 ] ]
       ]
     }
  ]
} */
typedef union {                                                                 // union of all geoJSON types
  geoJsonPoint_t point;
  geoJsonLineString_t linestring;
  geoJsonPolygon_t polygon;
  geoJsonMulLineStr_t multiline;
  geoJsonMulPolygon_t multipoly;
} geoJson_type_union_t;

#if defined(D_FT900)                                                            // geoJson GeometryCollection object
typedef struct GEOJSONPACKED {
   uint8_t type;
   geoJson_type_union_t gjType;
} geoJsonRW_t;
#else
GEOJSONPACKED(
typedef struct {
   uint8_t type;
   geoJson_type_union_t gjType;
}) geoJsonRW_t;                                                   
#endif   

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* -- geo JSON -- */