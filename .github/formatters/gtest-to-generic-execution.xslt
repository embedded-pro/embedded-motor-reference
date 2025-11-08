<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
    <xsl:output method="xml" indent="yes" omit-xml-declaration="yes"/>

    <!-- Use Muenchian grouping to group by filename; https://en.wikipedia.org/wiki/XSLT/Muenchian_grouping -->
    <xsl:key name="testcases-by-file" match="*/testsuite/testcase" use="@file" />

    <!-- Template to convert absolute paths to relative paths -->
    <xsl:template name="make-relative-path">
        <xsl:param name="path"/>
        <xsl:choose>
            <!-- If path contains /application/, extract from application onwards -->
            <xsl:when test="contains($path, '/application/')">
                <xsl:value-of select="concat('application/', substring-after($path, '/application/'))"/>
            </xsl:when>
            <!-- Otherwise return the path as-is -->
            <xsl:otherwise>
                <xsl:value-of select="$path"/>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

    <xsl:template match="/">
            <xsl:for-each select="*/testsuite/testcase[count(. | key('testcases-by-file', @file)[1]) = 1]">
                <xsl:variable name="current-grouping-key" select="@file"/>
                <xsl:variable name="current-group" select="key('testcases-by-file', $current-grouping-key)"/>

                <!-- Convert absolute path to relative path -->
                <xsl:variable name="relative-path">
                    <xsl:call-template name="make-relative-path">
                        <xsl:with-param name="path" select="$current-grouping-key"/>
                    </xsl:call-template>
                </xsl:variable>

                <file path="{$relative-path}">
                    <xsl:for-each select="$current-group">
                        <testCase name="{@name}" duration="{1000 * number(@time)}">
                            <xsl:if test="contains(@status, 'notrun')">
                                <skipped message="Disabled">Skipped disabled test</skipped>
                            </xsl:if>
                        </testCase>
                    </xsl:for-each>
                </file>
            </xsl:for-each>
    </xsl:template>

</xsl:stylesheet>
