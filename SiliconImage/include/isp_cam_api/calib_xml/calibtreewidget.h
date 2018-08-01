/******************************************************************************
 *
 * The copyright in this software is owned by Rockchip and/or its licensors.
 * This software is made available subject to the conditions of the license 
 * terms to be determined and negotiated by Rockchip and you.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHP AND/OR 
 * ITS LICENSORS DISCLAIMS ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH 
 * RESPECT TO SUCH SOFTWARE, WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, 
 * INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, 
 * MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Except as expressively authorized by Rockchip and/or its licensors, you may not 
 * (a) disclose, distribute, sell, sub-license, or transfer this software to any third party, 
 * in whole or part; (b) modify this software, in whole or part; (c) decompile, reverse-engineer, 
 * dissemble, or attempt to derive any source code from the software.
 *
 *****************************************************************************/
/**
 * @file    calibtreewidget.h
 *
 *
 *****************************************************************************/
#ifndef __CALIB_TREEWIDGET_H__
#define __CALIB_TREEWIDGET_H__

#include <QDomDocument>
#include <QHash>
#include <QIcon>
#include <QTreeWidget>
#include <QtXml>



class CalibTreeWidget : public QTreeWidget
{
    Q_OBJECT

public:
    CalibTreeWidget( QWidget *parent = NULL );
    ~CalibTreeWidget( );

    bool readFile( QFile * );

    QString getCreationDate( void );
    QString getCreator( void );
    QString getSensorName( void );
    QString getSampleName( void );
    QString getGeneratorVersion( void );

private slots:
    void updateDomElement( QTreeWidgetItem *, int );
    void doubleClickedDomElement( QTreeWidgetItem *, int );
    void clickedDomElement( QTreeWidgetItem *, int );

private:
    typedef bool (CalibTreeWidget::*parseCellContent)(const QDomElement &, QTreeWidgetItem *);

    // parse helper
    bool parseEntryCell( const QDomElement &, QTreeWidgetItem *, int, parseCellContent );

    // populate Header
    bool populateEntryHeader( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryResolution( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor
    bool populateEntrySensor( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-AEC
    bool populateEntryAec( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAecEcm( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAecEcmPriorityScheme( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-AWB
    bool populateEntryAwb( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAwbGlobals( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAwbIllumination( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAwbIlluminationAlsc( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryAwbIlluminationAcc( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-LSC
    bool populateEntryLsc( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-CC
    bool populateEntryCc( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-BLS
    bool populateEntryBls( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-CAC
    bool populateEntryCac( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-DPF
    bool populateEntryDpf( const QDomElement &, QTreeWidgetItem * );

    // populate Sensor-DPCC
    bool populateEntryDpcc( const QDomElement &, QTreeWidgetItem * );
    bool populateEntryDpccRegisters( const QDomElement &, QTreeWidgetItem * );

    // populate System 
    bool populateEntrySystem( const QDomElement &, QTreeWidgetItem * );

    QTreeWidgetItem *createTreeItem( const QDomElement &, QTreeWidgetItem *  = NULL );

private:
    QDomDocument                            doc;
    QDomElement                             root;

    QHash<QTreeWidgetItem *, QDomElement>   domElementForItem;

    QIcon                                   headerIcon;
    QIcon                                   sensorIcon;
    QIcon                                   fileIcon;

    QString                                 m_creationdate;
    QString                                 m_creator;
    QString                                 m_sensorname;
    QString                                 m_samplename;
    QString                                 m_generatorversion;
};


#endif /* __CALIB_TREEWIDGET_H__ */
