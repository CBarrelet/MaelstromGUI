#pragma once
#include "config.h"
#include "iostream"
#include "string"
#include "gdal_priv.h"
#include "cpl_conv.h"
#include "gdalwarper.h"
#include "stdlib.h"



class Geotiff {

private: // NOTE: "private" keyword is redundant here.  
         // we place it here for emphasis. Because these
         // variables are declared outside of "public", 
         // they are private. 

    const char* filename;        // name of Geotiff
    GDALDataset* geotiffDataset; // Geotiff GDAL datset object. 
    double geotransform[6];      // 6-element geotranform array.
    int dimensions[3];           // X,Y, and Z dimensions. 
    int NROWS, NCOLS, NLEVELS;     // dimensions of data in Geotiff. 

public:

    // define constructor function to instantiate object
    // of this Geotiff class. 
    Geotiff(const char* tiffname) {
        filename = tiffname;
        GDALAllRegister();

        // set pointer to Geotiff dataset as class member.  
        geotiffDataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);

        // set the dimensions of the Geotiff 
        NROWS = GDALGetRasterYSize(geotiffDataset);
        NCOLS = GDALGetRasterXSize(geotiffDataset);
        NLEVELS = GDALGetRasterCount(geotiffDataset);

        std::cout << NROWS << std::endl;
        std::cout << NCOLS << std::endl;
        std::cout << NLEVELS << std::endl;
       

    }



    // define destructor function to close dataset, 
    // for when object goes out of scope or is removed
    // from memory. 
    ~Geotiff() {
        // close the Geotiff dataset, free memory for array.  
        GDALClose(geotiffDataset);
        GDALDestroyDriverManager();
    }

    const char* GetFileName() {
        /*
         * function GetFileName()
         * This function returns the filename of the Geotiff.
         */
        return filename;
    }

    const char* GetProjection() {
        /* function const char* GetProjection():
         *  This function returns a character array (string)
         *  for the projection of the geotiff file. Note that
         *  the "->" notation is used. This is because the
         *  "geotiffDataset" class variable is a pointer
         *  to an object or structure, and not the object
         *  itself, so the "." dot notation is not used.
         */
        return geotiffDataset->GetProjectionRef();
    }

    double* GetGeoTransform() {
        /*
         * function double *GetGeoTransform()
         *  This function returns a pointer to a double that
         *  is the first element of a 6 element array that holds
         *  the geotransform of the geotiff.
         */
        geotiffDataset->GetGeoTransform(geotransform);
        return geotransform;
    }
    
    std::vector<std::vector<float>> readWindowRaster() {
        
        double* geotransform = GetGeoTransform();
        double originX = geotransform[0];
        double originY = geotransform[3];
        double pixelWidth = geotransform[1];
        double pixelHeight = geotransform[5];
        double window_Xorigin = 293083; //289628  ; //in meters
        double window_Yorigin = 5034918; //5033699  ;
        double xOffset = int((window_Xorigin - originX) / pixelWidth);
        double yOffset = int((window_Yorigin - originY) / pixelHeight); //center coordinates
        //size in meters
        double win_xsize = 10;
        double win_ysize = 10;
        int winx = int(win_xsize / pixelWidth);
        int winy = -int(win_ysize / pixelHeight);
        //get the projection
        //const char* projection = GetProjection();
        
        
        float** band = GetRasterBand(1); //TODO suivant la vitesse d'execution, ne faire qu'une fois au début du programme et stocker la band dans un argument de la classe
        std::cout << "bandok" << std::endl;

       
        //band->RasterIO(GF_Read, xOffset - winx / 2, yOffset - winy / 2, winx, winy,
        //    blockData, pixelWidth, pixelHeight, GDT_Float32, 0, 0);
       
        std::vector<std::vector<float>> vect;
       
        int beginy = yOffset - (winy / 2);
        int endy = yOffset + (winy / 2);
        int beginx = xOffset - (winx / 2);
        int endx = xOffset + (winx / 2);
        std::cout << "beginy " << beginy << " endy " << endy << " beginx " << beginx << "  endx " << endx << std::endl;
        for (int i = beginy; i < endy; i++) {
            std::vector<float> lign;
            std::cout << i << std::endl;
            for (int j = beginx; j <endx ; j++) {
                lign.push_back(band[i][j]);
            }
            vect.push_back(lign);
        }
        
        return vect;
          
    }

    double GetNoDataValue() {
        /*
         * function GetNoDataValue():
         *  This function returns the NoDataValue for the Geotiff dataset.
         *  Returns the NoData as a double.
         */
        return (double)geotiffDataset->GetRasterBand(1)->GetNoDataValue();
    }

    int* GetDimensions() {
        /*
         * int *GetDimensions():
         *
         *  This function returns a pointer to an array of 3 integers
         *  holding the dimensions of the Geotiff. The array holds the
         *  dimensions in the following order:
         *   (1) number of columns (x size)
         *   (2) number of rows (y size)
         *   (3) number of bands (number of bands, z dimension)
         */
        dimensions[0] = NROWS;
        dimensions[1] = NCOLS;
        dimensions[2] = NLEVELS;
        return dimensions;
    }


    std::vector<std::vector<float>> GetVectorBand() {
        int* dim = GetDimensions();
        float** pointerBand = GetRasterBand(1);
        std::vector<std::vector<float>> vectorBand;
        for (int i = 0; i < dim[0]; i++) {
            std::vector<float> lign;
            for (int j = 0; j < dim[1]; j++) {
                lign.push_back(pointerBand[i][j]);

            }
            vectorBand.push_back(lign);
        }
        return vectorBand;

    }


    float** GetRasterBand(int z) {

        /*
         * function float** GetRasterBand(int z):
         * This function reads a band from a geotiff at a
         * specified vertical level (z value, 1 ...
         * n bands). To this end, the Geotiff's GDAL
         * data type is passed to a switch statement,
         * and the template function GetArray2D (see below)
         * is called with the appropriate C++ data type.
         * The GetArray2D function uses the passed-in C++
         * data type to properly read the band data from
         * the Geotiff, cast the data to float**, and return
         * it to this function. This function returns that
         * float** pointer.
         */

        float** bandLayer = new float* [NROWS];
        switch (GDALGetRasterDataType(geotiffDataset->GetRasterBand(z))) {
        case 0:
            return NULL; // GDT_Unknown, or unknown data type.
        case 1:
            // GDAL GDT_Byte (-128 to 127) - unsigned  char
            return GetArray2D<unsigned char>(z, bandLayer);
        case 2:
            // GDAL GDT_UInt16 - short
            return GetArray2D<unsigned short>(z, bandLayer);
        case 3:
            // GDT_Int16
            return GetArray2D<short>(z, bandLayer);
        case 4:
            // GDT_UInt32
            return GetArray2D<unsigned int>(z, bandLayer);
        case 5:
            // GDT_Int32
            return GetArray2D<int>(z, bandLayer);
        case 6:
            // GDT_Float32
            return GetArray2D<float>(z, bandLayer);
        case 7:
            // GDT_Float64
            return GetArray2D<double>(z, bandLayer);
        default:
            break;
        }
        return NULL;
    }

    template<typename T>
    float** GetArray2D(int layerIndex, float** bandLayer) {

        /*
         * function float** GetArray2D(int layerIndex):
         * This function returns a pointer (to a pointer)
         * for a float array that holds the band (array)
         * data from the geotiff, for a specified layer
         * index layerIndex (1,2,3... for GDAL, for Geotiffs
         * with more than one band or data layer, 3D that is).
         *
         * Note this is a template function that is meant
         * to take in a valid C++ data type (i.e. char,
         * short, int, float), for the Geotiff in question
         * such that the Geotiff band data may be properly
         * read-in as numbers. Then, this function casts
         * the data to a float data type automatically.
         */

         // get the raster data type (ENUM integer 1-12, 
         // see GDAL C/C++ documentation for more details)        
        GDALDataType bandType = GDALGetRasterDataType(
            geotiffDataset->GetRasterBand(layerIndex));

        // get number of bytes per pixel in Geotiff
        int nbytes = GDALGetDataTypeSizeBytes(bandType);

        // allocate pointer to memory block for one row (scanline) 
        // in 2D Geotiff array.  
        T* rowBuff = (T*)CPLMalloc(nbytes * NCOLS);

        for (int row = 0; row < NROWS; row++) {     // iterate through rows

          // read the scanline into the dynamically allocated row-buffer       
            CPLErr e = geotiffDataset->GetRasterBand(layerIndex)->RasterIO(
                GF_Read, 0, row, NCOLS, 1, rowBuff, NCOLS, 1, bandType, 0, 0);
            if (!(e == 0)) {
                std::cout << "Warning: Unable to read scanline in Geotiff!" << std::endl;
                exit(1);
            }

            bandLayer[row] = new float[NCOLS];
            for (int col = 0; col < NCOLS; col++) { // iterate through columns
                bandLayer[row][col] = (float)rowBuff[col];
            }
        }
        CPLFree(rowBuff);
        return bandLayer;
    }

};