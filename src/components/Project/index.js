/*
 * @Author: Ceoifung
 * @Date: 2023-05-05 10:58:46
 * @LastEditors: Ceoifung
 * @LastEditTime: 2023-05-05 12:36:47
 * @Description: XiaoRGEEK All Rights Reserved. Copyright © 2023
 */
import React from 'react';
import { Link } from 'react-router-dom'

/**
 * 两列卡片
 * @param {*} param0 children, children, showChildren1: 是否展示第二个模组
 * @returns 
 */
const ProjectDoubleCard = ({ children, children1, showChildren1 = true }) => (
    <div class="father-box">
        <Link
            class="box-class" to={children.link ? children.link : "/"}>
            <div>
                <span class="box-class-title">{children.title}</span><br />
                <span>{children.description}</span>
            </div>
        </Link>
        {
            showChildren1 ? <Link
                class="box-class" to={children1.link ? children1.link : "/"}>
                <div>
                    <span class="box-class-title">{children1.title}</span><br />
                    <span>{children1.description}</span>
                </div>
            </Link> : null
        }

    </div>

);

/**
 * 整个布满布局的卡片
 * @param {*} param0 参数，需要包含title, link, description
 * @returns 布局
 */
const ProjectCard = ({ children }) => (
    <Link
        class="box-class" to={children.link ? children.link : "/"}>
        <div>
            <span class="box-class-title">{children.title}</span><br />
            <span>{children.description}</span>
        </div>
    </Link>
)

export {
    ProjectCard, ProjectDoubleCard
}