#ifndef INSTA360_STITCHER_H__
#define INSTA360_STITCHER_H__
#include <memory>
#include <string>
#include <vector>

#include "common.h"

#define MEDIASDK_API
#if WIN32
    #ifdef USE_EXPORTDLL
        #define MEDIASDK_API _declspec(dllexport)
    #else
        #define MEDIASDK_API _declspec(dllimport)
    #endif // USE_EXPORTDLL
#endif

namespace ins_media {
    class  StitcherImp;
    /**
     * \class VideoStitcher
     * \brief A class that stitch video, only support the formats that maked by Insta360 camera, for example *.insv .
     */
    class MEDIASDK_API VideoStitcher {

        public:
            VideoStitcher();
            ~VideoStitcher() = default;

        public:
            /**
             * \brief set the path of the source that you wanted.
             * \param input_paths if the source is the kind that ‘5.7k’, the size of the vector is 2，1 othersize                 
             */
            void SetInputPath(std::vector<std::string>& input_paths);

            /**
             * \brief set the path of the output that you wanted
             */
            void SetOutputPath(const std::string& output_path);

            /**
             * \brief set the video bitrate of the output file
             */
            void SetOutputBitRate(int64_t bitRate);

            /**
             * \brief set the video resolution of the output file，default size of source
             */
            void SetOutputSize(int width, int height);
            
            /**
             * \brief Whether to turn on FlowState
             * \param enable true is on and false is off defaule off
             */
            void EnableFlowState(bool enable);

            /**
             * \brief set the type of stitch,
             * \param type  it has three kind that has template,optflow and dynamicStitch
             * \            template is fastest and optflow is slowest.
             */
            void SetStitchType(STITCH_TYPE type);

            /**
             *  \brief set the callback that can tell the client the process of stitch
             *  \param callback used for get process the process
             *  \param lcontent the context
             */
            void SetStitchProgressCallback(stitch_process_callback callback, long lcontent); 

            /**
             *  \brief set the callback that can tell the client the error if stitcher goes error 
             *  \param callback used for get err info of the process.
             *  \param lcontent the context
             */
            void SetStitchStateCallback(stitch_error_callback callback, long lcontent);

            /**
             *  \brief start stitch
             */
            void StartStitch();

            /**
             *  \brief cancel stitch
             */
            bool CancelStitch();

        private:
            std::shared_ptr<StitcherImp> imp_;
    };

    /**
     * \class ImageStitcher
     * \brief A class that stitch image, only support the formats that maked by Insta360 camera, for example *.insp
     */
    class MEDIASDK_API ImageStitcher {
        
    public:
        ImageStitcher();
        ~ImageStitcher() = default;

    public:
         /**
             * \brief set the path of the source that you wanted.
             * \param input_path                
             */
            void SetInputPath(std::vector<std::string>& input_paths);

            /**
             * \brief set the path of the output that you wanted
             */
            void SetOutputPath(const std::string& output_path);

            /**
             * \brief set the video resolution of the output file
             */
            void SetOutputSize(int width, int height);

            /**
            * \brief set the type of HDR
            * \ param type the type of hdr, default ImageHdr_NONE that turn off HDR 
            * \            SingleImageHdr is used for singleImage
            * \            MultiImageHdr_mbb and MultiImageHdr_mpl are used for multiple images
            */
            void SetHDRType(HDR_TYPE type);

            /**
             * \brief Whether to turn on FlowState
             * \param enable true is on and false is off defaule off
             */
            void EnableFlowState(bool enable);

            /**
             * \brief set the type of stitch,
             * \param type  it has three kind that has template,optflow and dynamicStitch
             * \            template is fastest and optflow is slowest.
             */
            void SetStitchType(STITCH_TYPE type);

            /**
             * \brief start stitch, this function is called synchronously.
             */
            bool Stitch();
    private:
        std::shared_ptr<StitcherImp> imp_;
    };
} // namespace ins_stitche 


#endif /*INSTA360_STITCHER_H__*/