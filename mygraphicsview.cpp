#include "mygraphicsview.h"

MyGraphicsView::MyGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
    , scene(this)
{
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	setScene(&scene);
	
    scene.setBackgroundBrush(Qt::black);
    scene.addItem(&map);
    scene.addRect(0, 0, robot_config.robot_map_size, robot_config.robot_map_size, QPen(QBrush(Qt::yellow), 3));

    target_pose = cv::Vec3d(0,0,0);
}

MyGraphicsView::~MyGraphicsView()
{
    scene.removeItem(&map);
}

void MyGraphicsView::wheelEvent(QWheelEvent * ev)
{
	// Scale the view / do the zoom
	double scaleFactor = 1.05;
	if (ev->delta() > 0)
	{
		// Zoom in
		scale(scaleFactor, scaleFactor);
	}
	else
	{
		// Zooming out
		scale(1.0 / scaleFactor, 1.0 / scaleFactor);
	}

	//QGraphicsView::wheelEvent(ev);
}

void MyGraphicsView::mousePressEvent(QMouseEvent * ev)
{
	if (ev->button() == Qt::LeftButton)
	{
        QPointF pt = mapToScene(ev->pos());

        // for guide line
        pt0 = pt;
        line.setPen(QPen(Qt::yellow, 1, Qt::DotLine));
        line.setLine(QLineF(pt, pt));
        scene.addItem(&line);

        // for desired robot pose
        x0 = -(pt.y()-(robot_config.robot_map_size/2))*robot_config.robot_grid_size;
        y0 = -(pt.x()-(robot_config.robot_map_size/2))*robot_config.robot_grid_size;
		isDragL = true;
	}
	
	if (ev->button() == Qt::RightButton)
	{
        QPointF pt = mapToScene(ev->pos());
		isDragR = true;
	}

	QGraphicsView::mousePressEvent(ev);
}

void MyGraphicsView::mouseMoveEvent(QMouseEvent * ev)
{
	if (isDragL)
	{
        QPointF pt = mapToScene(ev->pos());        
        line.setLine(QLineF(pt0, pt));
	}
	
	if (isDragR)
	{
        QPointF pt = mapToScene(ev->pos());
	}

	QGraphicsView::mouseMoveEvent(ev);
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent * ev)
{
	if (ev->button() == Qt::LeftButton)
	{
		isDragL = false;
        scene.removeItem(&line);

        QPointF pt = mapToScene(ev->pos());
        double x1 = -(pt.y()-(robot_config.robot_map_size/2))*robot_config.robot_grid_size;
        double y1 = -(pt.x()-(robot_config.robot_map_size/2))*robot_config.robot_grid_size;
        double x = x0;
        double y = y0;
        double th = std::atan2(y1-y0, x1-x0);

        if(!isGrab)
        {
            target_pose = cv::Vec3d(x, y, th);
            emit pose_clicked(x, y, th);
            std::cout<<"x : "<<x<<",y : "<<y<<",theta :"<<th*R2D<<std::endl;
        }
	}

	if (ev->button() == Qt::RightButton)
	{
		isDragR = false;
        QPointF pt = mapToScene(ev->pos());
	}

	QGraphicsView::mouseReleaseEvent(ev);
}

void MyGraphicsView::keyPressEvent(QKeyEvent * ev)
{
	if (ev->isAutoRepeat())
	{
		return;
	}

	if (ev->key() == Qt::Key_Control)
	{
        isGrab = true;
		setDragMode(ScrollHandDrag);
		return;
	}
}

void MyGraphicsView::keyReleaseEvent(QKeyEvent * ev)
{
	if (ev->isAutoRepeat())
	{
		return;
	}

	if (ev->key() == Qt::Key_Control)
	{
		setDragMode(NoDrag);
        isGrab = false;
		return;
	}
}
