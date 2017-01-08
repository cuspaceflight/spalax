#include "wmm_util.h"
#include <string.h>
#include <stdbool.h>
#include <GeomagnetismHeader.h>
#include "component_state.h"
#include "messaging.h"

#if defined MESSAGING_OS_STD
// Don't need to do anything
#elif defined MESSAGING_OS_CHIBIOS
// Override allocators
#define HEAP_SIZE (1024 & ~0x0F) + 16

static volatile char memory_buffer[HEAP_SIZE] MEMORY_BUFFER_ATTRIBUTES;
static memory_heap_t memory_heap;

void* geo_malloc(size_t n) {
    return chHeapAlloc(&memory_heap, n);
}

void* geo_calloc(size_t num, size_t size) {
    // NB: No attempt is made to correctly handle overflow
    return chHeapAlloc(&memory_heap, num * size);
}

void geo_free(void* ptr) {
    chHeapFree(ptr);
}

#else
#error Unrecognised Messaging OS
#endif

typedef struct model_parameter_t {
    int n;
    int m;
    double gnm;
    double hnm;
    double dgnm;
    double dhnm;
} model_parameter_t;

#define EPOCHS 1
#define EPOCH 20

static const double wmm_epoch = 2015.0;
static const int wmm_nmax = 12;
static const int wmm_num_terms = CALCULATE_NUMTERMS(12);
static const char* wmm_model_name = "WMM-2015"; // Must be less than 31 characters
static const model_parameter_t wmm_parameters[] = {
    {1,0,-29438.5,0.0,10.7,0.0},
    {1,1,-1501.1,4796.2,17.9,-26.8},
    {2,0,-2445.3,0.0,-8.6,0.0},
    {2,1,3012.5,-2845.6,-3.3,-27.1},
    {2,2,1676.6,-642.0,2.4,-13.3},
    {3,0,1351.1,0.0,3.1,0.0},
    {3,1,-2352.3,-115.3,-6.2,8.4},
    {3,2,1225.6,245.0,-0.4,-0.4},
    {3,3,581.9,-538.3,-10.4,2.3},
    {4,0,907.2,0.0,-0.4,0.0},
    {4,1,813.7,283.4,0.8,-0.6},
    {4,2,120.3,-188.6,-9.2,5.3},
    {4,3,-335.0,180.9,4.0,3.0},
    {4,4,70.3,-329.5,-4.2,-5.3},
    {5,0,-232.6,0.0,-0.2,0.0},
    {5,1,360.1,47.4,0.1,0.4},
    {5,2,192.4,196.9,-1.4,1.6},
    {5,3,-141.0,-119.4,0.0,-1.1},
    {5,4,-157.4,16.1,1.3,3.3},
    {5,5,4.3,100.1,3.8,0.1},
    {6,0,69.5,0.0,-0.5,0.0},
    {6,1,67.4,-20.7,-0.2,0.0},
    {6,2,72.8,33.2,-0.6,-2.2},
    {6,3,-129.8,58.8,2.4,-0.7},
    {6,4,-29.0,-66.5,-1.1,0.1},
    {6,5,13.2,7.3,0.3,1.0},
    {6,6,-70.9,62.5,1.5,1.3},
    {7,0,81.6,0.0,0.2,0.0},
    {7,1,-76.1,-54.1,-0.2,0.7},
    {7,2,-6.8,-19.4,-0.4,0.5},
    {7,3,51.9,5.6,1.3,-0.2},
    {7,4,15.0,24.4,0.2,-0.1},
    {7,5,9.3,3.3,-0.4,-0.7},
    {7,6,-2.8,-27.5,-0.9,0.1},
    {7,7,6.7,-2.3,0.3,0.1},
    {8,0,24.0,0.0,0.0,0.0},
    {8,1,8.6,10.2,0.1,-0.3},
    {8,2,-16.9,-18.1,-0.5,0.3},
    {8,3,-3.2,13.2,0.5,0.3},
    {8,4,-20.6,-14.6,-0.2,0.6},
    {8,5,13.3,16.2,0.4,-0.1},
    {8,6,11.7,5.7,0.2,-0.2},
    {8,7,-16.0,-9.1,-0.4,0.3},
    {8,8,-2.0,2.2,0.3,0.0},
    {9,0,5.4,0.0,0.0,0.0},
    {9,1,8.8,-21.6,-0.1,-0.2},
    {9,2,3.1,10.8,-0.1,-0.1},
    {9,3,-3.1,11.7,0.4,-0.2},
    {9,4,0.6,-6.8,-0.5,0.1},
    {9,5,-13.3,-6.9,-0.2,0.1},
    {9,6,-0.1,7.8,0.1,0.0},
    {9,7,8.7,1.0,0.0,-0.2},
    {9,8,-9.1,-3.9,-0.2,0.4},
    {9,9,-10.5,8.5,-0.1,0.3},
    {10,0,-1.9,0.0,0.0,0.0},
    {10,1,-6.5,3.3,0.0,0.1},
    {10,2,0.2,-0.3,-0.1,-0.1},
    {10,3,0.6,4.6,0.3,0.0},
    {10,4,-0.6,4.4,-0.1,0.0},
    {10,5,1.7,-7.9,-0.1,-0.2},
    {10,6,-0.7,-0.6,-0.1,0.1},
    {10,7,2.1,-4.1,0.0,-0.1},
    {10,8,2.3,-2.8,-0.2,-0.2},
    {10,9,-1.8,-1.1,-0.1,0.1},
    {10,10,-3.6,-8.7,-0.2,-0.1},
    {11,0,3.1,0.0,0.0,0.0},
    {11,1,-1.5,-0.1,0.0,0.0},
    {11,2,-2.3,2.1,-0.1,0.1},
    {11,3,2.1,-0.7,0.1,0.0},
    {11,4,-0.9,-1.1,0.0,0.1},
    {11,5,0.6,0.7,0.0,0.0},
    {11,6,-0.7,-0.2,0.0,0.0},
    {11,7,0.2,-2.1,0.0,0.1},
    {11,8,1.7,-1.5,0.0,0.0},
    {11,9,-0.2,-2.5,0.0,-0.1},
    {11,10,0.4,-2.0,-0.1,0.0},
    {11,11,3.5,-2.3,-0.1,-0.1},
    {12,0,-2.0,0.0,0.1,0.0},
    {12,1,-0.3,-1.0,0.0,0.0},
    {12,2,0.4,0.5,0.0,0.0},
    {12,3,1.3,1.8,0.1,-0.1},
    {12,4,-0.9,-2.2,-0.1,0.0},
    {12,5,0.9,0.3,0.0,0.0},
    {12,6,0.1,0.7,0.1,0.0},
    {12,7,0.5,-0.1,0.0,0.0},
    {12,8,-0.4,0.3,0.0,0.0},
    {12,9,-0.4,0.2,0.0,0.0},
    {12,10,0.2,-0.9,0.0,0.0},
    {12,11,-0.9,-0.2,0.0,0.0},
    {12,12,0.0,0.7,0.0,0.0}
};


static MAGtype_MagneticModel* magnetic_models[EPOCHS];
static MAGtype_MagneticModel* timed_magnetic_model;
static MAGtype_Ellipsoid ellipsoid;
static MAGtype_Geoid geoid;

void wmm_util_init(double model_time) {
    COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_initializing);

    if (model_time < wmm_epoch || model_time > wmm_epoch + 5)
        COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_error);

#if defined MESSAGING_OS_CHIBIOS
    chHeapObjectInit(&memory_heap, (void*)memory_buffer, HEAP_SIZE);
#endif
    MAGtype_MagneticModel* model = magnetic_models[0] = MAG_AllocateModelMemory(wmm_num_terms);

    if (model == NULL) {
        COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_error);
        return;
    }

    model->nMax = wmm_nmax;
    model->nMaxSecVar = wmm_nmax;

    model->Main_Field_Coeff_H[0] = 0.0;
    model->Main_Field_Coeff_G[0] = 0.0;
    model->Secular_Var_Coeff_H[0] = 0.0;
    model->Secular_Var_Coeff_G[0] = 0.0;
    strcpy(model->ModelName, wmm_model_name);
    model->epoch = wmm_epoch;

    for (int i = 0; i < sizeof(wmm_parameters) / sizeof(model_parameter_t); i++) {
        const model_parameter_t* p = &wmm_parameters[i]; 
        if(p->m <= p->n) {
            int index = (p->n * (p->n + 1) / 2 + p->m);
            model->Main_Field_Coeff_G[index] = p->gnm;
            model->Secular_Var_Coeff_G[index] = p->dgnm;
            model->Main_Field_Coeff_H[index] = p->hnm;
            model->Secular_Var_Coeff_H[index] = p->dhnm;
        }
    }

    model->CoefficientFileEndDate = model->epoch + 5;

    COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_initializing);

    timed_magnetic_model = MAG_AllocateModelMemory(wmm_num_terms);

    if (timed_magnetic_model == NULL) {
        COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_error);
    }

    MAGtype_Date date;
    date.DecimalYear = model_time;

    MAG_TimelyModifyMagneticModel(date, magnetic_models[0], timed_magnetic_model); 

    COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_initializing);

    MAG_SetDefaults(&ellipsoid, &geoid); /* Set default values and constants */
        
    /* Set EGM96 Geoid parameters */
    //geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    //geoid.Geoid_Initialized = 1;

    COMPONENT_STATE_UPDATE(avionics_component_world_mag_model, state_ok);
}

void wmm_util_get_magnetic_field(float latitude, float longitude, float elevation, MagneticFieldParams* out) {
    MAGtype_CoordSpherical coordSpherical;    
    MAGtype_CoordGeodetic coordGeodetic;
    MAGtype_GeoMagneticElements geoMagneticElements;

    coordGeodetic.phi = latitude;
    coordGeodetic.lambda = longitude;

    // Height above the WGS84 ellipsoid
    coordGeodetic.HeightAboveEllipsoid = elevation;
    // Height above the EGM96 Geoid model
    coordGeodetic.HeightAboveGeoid = 0;
    coordGeodetic.UseGeoid = 0;

    MAG_GeodeticToSpherical(ellipsoid, coordGeodetic, &coordSpherical);
    
    MAG_Geomag(ellipsoid, coordSpherical, coordGeodetic, timed_magnetic_model, &geoMagneticElements);
    MAG_CalculateGridVariation(coordGeodetic, &geoMagneticElements);

    out->declination = (float) geoMagneticElements.Decl;
    out->inclination = (float) geoMagneticElements.Incl;
    out->field_strength = (float) geoMagneticElements.F;
    out->horizontal_field_strength = (float) geoMagneticElements.H;
    out->field_vector[0] = (float) geoMagneticElements.X;
    out->field_vector[1] = (float) geoMagneticElements.Y;
    out->field_vector[2] = (float) geoMagneticElements.Z;
}

bool wmm_util_get_year(int year, int month, int day, double *out) {
    MAGtype_Date date = {year, month, day};
    if (!MAG_DateToYear(&date,NULL))
        return false;
    *out = date.DecimalYear;
    return true;
}
