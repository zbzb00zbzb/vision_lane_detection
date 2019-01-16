/**
 * @file IPMTransformation.h
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date June 04, 2018
 */

#ifndef LANE_LINE_IPMTRANSFORMATION_H
#define LANE_LINE_IPMTRANSFORMATION_H

#include <memory>

#include "lane_line/common.h"
#include "lane_line/IPMParameters.h"
#include "lane_line/Transformation.h"

namespace tt
{


class IPMTransformation : public Transformation
{
 public:
    IPMTransformation() = default;
    explicit IPMTransformation(const IPMParametersProto &param);

    cv::Mat computeIPMImage(
            const cv::Mat &raw_image,
            int flags = cv::INTER_LINEAR) const;
    /**
     * compute the raw image from the ipm image.
     * @param ipm_image
     * @param size  Size of the raw image.
     * @return the raw image
     */
    cv::Mat computeRawImage(const cv::Mat &ipm_image,
                            const cv::Size& size) const;

    static std::shared_ptr<IPMTransformation>
    create(const IPMParametersProto &param);

    void init(const IPMParametersProto &param);
    void init(const IPMParameters &param);
    void initWithPrecomputedH(const IPMParametersProto &param);
 public:
    const cv::Matx33f& getImageToIPM() const {return image_to_ipm_;}
    const cv::Matx33f& getIPMToImage() const {return ipm_to_image_;}

    IPMParametersProto& param() {return param_;}
    const IPMParametersProto& param() const {return param_;}
 private:
    cv::Matx33f image_to_ipm_;
    cv::Matx33f ipm_to_image_;

    IPMParametersProto param_;
};


}  // namespace tt

#endif  // LANE_LINE_IPMTRANSFORMATION_H
