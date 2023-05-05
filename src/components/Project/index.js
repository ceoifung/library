/*
 * @Author: Ceoifung
 * @Date: 2023-05-05 10:58:46
 * @LastEditors: Ceoifung
 * @LastEditTime: 2023-05-05 11:11:41
 * @Description: XiaoRGEEK All Rights Reserved. Copyright © 2023
 */
import React from 'react';
import {Link} from 'react-router-dom'
const ProjectCard = ({children, description, link}) => (
  <Link
    class="box-class" to={link?link:"/"}>
    <div>
        <span class="box-class-title">{children}</span><br/>
        <span>{description}</span>
    </div>
  </Link>
);

export {
    ProjectCard
}