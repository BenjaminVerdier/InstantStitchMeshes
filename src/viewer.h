/*
    viewer.h: Contains the graphical user interface of Instant Meshes

    This file is part of the implementation of

        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#pragma once

#include "glutil.h"
#include "widgets.h"
#include "hierarchy.h"
#include "field.h"
#include "bvh.h"
#include "meshstats.h"
#include <set>

using nanogui::Alignment;
using nanogui::Arcball;
using nanogui::BoxLayout;
using nanogui::Button;
using nanogui::CheckBox;
using nanogui::Color;
using nanogui::ComboBox;
using nanogui::GLFramebuffer;
using nanogui::GroupLayout;
using nanogui::ImagePanel;
using nanogui::Label;
using nanogui::MessageDialog;
using nanogui::Orientation;
using nanogui::Popup;
using nanogui::PopupButton;
using nanogui::ProgressBar;
using nanogui::Screen;
using nanogui::Slider;
using nanogui::TextBox;
using nanogui::ToolButton;
using nanogui::VScrollPanel;
using nanogui::Widget;
using nanogui::Window;
using nanogui::frustum;
using nanogui::lookAt;
using nanogui::project;
using nanogui::scale;
using nanogui::translate;
using nanogui::unproject;
using nanogui::utf8;

#include <stb_image.h>

class GLTexture {
public:
    using handleType = std::unique_ptr<uint8_t[], void(*)(void*)>;
    GLTexture() = default;
    GLTexture(const std::string& textureName)
        : mTextureName(textureName), mTextureId(0) {}

    GLTexture(const std::string& textureName, GLint textureId)
        : mTextureName(textureName), mTextureId(textureId) {}

    GLTexture(const GLTexture& other) = delete;
    GLTexture(GLTexture&& other) noexcept
        : mTextureName(std::move(other.mTextureName)),
        mTextureId(other.mTextureId) {
        other.mTextureId = 0;
    }
    GLTexture& operator=(const GLTexture& other) = delete;
    GLTexture& operator=(GLTexture&& other) noexcept {
        mTextureName = std::move(other.mTextureName);
        std::swap(mTextureId, other.mTextureId);
        return *this;
    }
    ~GLTexture() noexcept {
        if (mTextureId)
            glDeleteTextures(1, &mTextureId);
    }

    GLuint texture() const { return mTextureId; }
    const std::string& textureName() const { return mTextureName; }

    /**
    *  Load a file in memory and create an OpenGL texture.
    *  Returns a handle type (an std::unique_ptr) to the loaded pixels.
    */
    handleType load(const std::string& fileName) {
        if (mTextureId) {
            glDeleteTextures(1, &mTextureId);
            mTextureId = 0;
        }
        int force_channels = 0;
        int w, h, n;
        handleType textureData(stbi_load(fileName.c_str(), &w, &h, &n, force_channels), stbi_image_free);
        if (!textureData)
            throw std::invalid_argument("Could not load texture data from file " + fileName);
        glGenTextures(1, &mTextureId);
        glBindTexture(GL_TEXTURE_2D, mTextureId);
        GLint internalFormat;
        GLint format;
        switch (n) {
        case 1: internalFormat = GL_R8; format = GL_RED; break;
        case 2: internalFormat = GL_RG8; format = GL_RG; break;
        case 3: internalFormat = GL_RGB8; format = GL_RGB; break;
        case 4: internalFormat = GL_RGBA8; format = GL_RGBA; break;
        default: internalFormat = 0; format = 0; break;
        }
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, GL_UNSIGNED_BYTE, textureData.get());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        return textureData;
    }

private:
    std::string mTextureName;
    GLuint mTextureId;
};

struct CurvePoint;

class Viewer : public Screen {
public:
    Viewer(bool fullscreen, bool deterministic);
    virtual ~Viewer();

    bool mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
                                  int button, int modifiers);

    bool mouseButtonEvent(const Vector2i &p, int button, bool down,
                                  int modifiers);

    bool keyboardEvent(int key, int scancode, int action, int modifiers);

    bool scrollEvent(const Vector2i &p, const Vector2f &rel);

    void loadInput(std::string filename,
                   Float creaseAngle = std::numeric_limits<Float>::infinity(),
                   Float scale = -1, int face_count = -1, int vertex_count = -1,
                   int rosy = 4, int posy = 4, int knn_points = 10);

    void setSymmetry(int rosy, int posy);
    void setExtrinsic(bool extrinsic);

    void resetState();
    void loadState(std::string filename, bool compat = false);
    void saveState(std::string filename);
    void renderMitsuba();
    void setFloorPosition();
    void draw(NVGcontext *ctx);

protected:
    void extractMesh();
    void extractConsensusGraph();

    void drawContents();
    void drawOverlay();

    bool resizeEvent(const Vector2i &size);

    void refreshColors();

    void traceFlowLines();

    void refreshStrokes();

    void showProgress(const std::string &caption, Float value);

    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    void setLevel(int level);
    void setTargetScale(Float scale);
    void setTargetVertexCount(uint32_t v);
    void setTargetVertexCountPrompt(uint32_t v);

    bool createSmoothPath(const std::vector<Vector2i> &curve);

    void repaint();

    void setCreaseAnglePrompt(bool enabled, Float creaseAngle);
    void shareGLBuffers();
    bool refreshPositionSingularities();
    bool refreshOrientationSingularities();
    std::pair<Vector3f, Vector3f> singularityPositionAndNormal(uint32_t v) const;
    bool toolActive() const;

protected:
    struct CameraParameters {
        Arcball arcball;
        float zoom = 1.0f, viewAngle = 45.0f;
        float dnear = 0.05f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 5.0f);
        Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
        Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
        float modelZoom = 1.0f;
    };

    std::vector<std::pair<int, std::string>> mExampleImages;
    std::string mFilename;
    bool mDeterministic;
    bool mUseHalfFloats;

    /* Data being processed */
    std::map<uint32_t, uint32_t> mCreaseMap;
    std::set<uint32_t> mCreaseSet;
    VectorXb mNonmanifoldVertices;
    VectorXb mBoundaryVertices;
    MultiResolutionHierarchy mRes;
    Optimizer mOptimizer;
    BVH *mBVH;
    MeshStats mMeshStats;
    int mSelectedLevel;
    Float mCreaseAngle;
    Matrix4f mFloor;
    VectorXu mE2E;

    /* Painting tools */
    std::vector<Vector2i> mScreenCurve;
    std::vector<std::pair<uint32_t, std::vector<CurvePoint>>> mStrokes;

    /* Extraction result */
    MatrixXu mF_extracted;
    MatrixXf mV_extracted;
    MatrixXf mN_extracted, mNf_extracted;
	MatrixXf mQ_extracted;

    /* Camera / navigation / misc */
    CameraParameters mCamera;
    CameraParameters mCameraSnapshots[12];
    Vector2i mTranslateStart;
    bool mTranslate, mDrag;
    std::map<uint32_t, uint32_t> mOrientationSingularities;
    std::map<uint32_t, Vector2i> mPositionSingularities;
    bool mContinueWithPositions;

    /* Colors */
    Vector3f mSpecularColor, mBaseColor;
    Vector3f mInteriorFactor, mEdgeFactor0;
    Vector3f mEdgeFactor1, mEdgeFactor2;

    /* OpenGL objects */
    GLFramebuffer mFBO;
    SerializableGLShader mPointShader63, mPointShader24, mPointShader44;
    SerializableGLShader mMeshShader63, mMeshShader24, mMeshShader44;
    SerializableGLShader mOrientationFieldShader;
    SerializableGLShader mPositionFieldShader;
    SerializableGLShader mPositionSingularityShader;
    SerializableGLShader mOrientationSingularityShader;
    SerializableGLShader mFlowLineShader, mStrokeShader;
    SerializableGLShader mOutputMeshShader;
    SerializableGLShader mOutputMeshWireframeShader;
    bool mNeedsRepaint;
    uint32_t mDrawIndex;

    /* Stitch Meshing related*/
    SerializableGLShader mLabeledMesh_F;
    SerializableGLShader mAlignedMesh_F;
    SerializableGLShader mAlignedMesh_E;
    SerializableGLShader mStitchMeshing_F;
    SerializableGLShader mStitchMeshing_E;

    GLTexture mArrowTexture;
    GLTexture mUVTexture;

    /* GUI-related */
    enum Layers {
        InputMesh,
        InputMeshWireframe,
        FaceLabels,
        VertexLabels,
        FlowLines,
        OrientationField,
        OrientationFieldSingularities,
        PositionField,
        PositionFieldSingularities,
        BrushStrokes,
        OutputMesh,
        OutputMeshWireframe,
        LabeledMesh,
        AlignedMesh,
        StitchMesh,
        LayerCount
    };

    CheckBox *mLayers[LayerCount];
    ComboBox *mVisualizeBox, *mSymmetryBox;
    CheckBox *mExtrinsicBox, *mAlignToBoundariesBox;
    CheckBox *mCreaseBox, *mPureQuadBox, *mSplitNgonsBox;
    ProgressButton *mSolveOrientationBtn, *mSolvePositionBtn;
    Button *mHierarchyMinusButton, *mHierarchyPlusButton;
    Button *mSaveBtn, *mSwitchBtn, *mStitchMeshingBtn;
    PopupButton *mExportBtn;
    ToolButton *mOrientationComb, *mOrientationAttractor, *mOrientationScareBrush;
    ToolButton *mEdgeBrush, *mPositionAttractor, *mPositionScareBrush;
    TextBox *mHierarchyLevelBox, *mScaleBox, *mCreaseAngleBox;
    TextBox *mOrientationSingularityBox, *mPositionSingularityBox, *mSmoothBox;
    Slider *mScaleSlider, *mCreaseAngleSlider, *mSmoothSlider;
    Slider *mOrientationFieldSizeSlider, *mOrientationFieldSingSizeSlider;
    Slider *mPositionFieldSingSizeSlider, *mFlowLineSlider;
#ifdef VISUALIZE_ERROR
    Graph *mGraph;
#endif

    /* Progress display */
    std::function<void(const std::string &, Float)> mProgress;
    Window *mProgressWindow;
    ProgressBar *mProgressBar;
    Label *mProgressLabel;
    tbb::spin_mutex mProgressMutex;
    double mLastProgressMessage;
    double mOperationStart;
    uint32_t mOutputMeshFaces, mOutputMeshLines;
    uint32_t mFlowLineFaces, mStrokeFaces;
};
